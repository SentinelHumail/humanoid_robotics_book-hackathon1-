from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
from pydantic import BaseModel
from typing import Optional, List, Dict, Any

from database import init_database, save_chat
from vector_store import QdrantStore
from rag_service import RAGService
from config import OPENAI_API_KEY, GITHUB_PAGES_URL, ALLOWED_ORIGINS
from openai import OpenAI


# Pydantic models for request/response
class ChatRequest(BaseModel):
    user_query: str
    user_id: str
    selected_text: Optional[str] = None


class ChatResponse(BaseModel):
    answer: str
    sources: List[Dict[str, Any]]


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup event
    init_database()  # Initialize database tables
    app.state.qdrant_store = QdrantStore()  # Initialize Qdrant store
    app.state.qdrant_store.init_collection()  # Initialize Qdrant collection
    app.state.rag_service = RAGService()  # Initialize RAG service with alternative AI providers
    yield
    # Shutdown event (if needed)


app = FastAPI(title="RAG Chatbot API", lifespan=lifespan)

# Configure CORS middleware with flexible origins for deployment
# Get allowed origins from environment variable, with defaults for local development
allowed_origins_env = ALLOWED_ORIGINS
if allowed_origins_env:
    allowed_origins = [origin.strip() for origin in allowed_origins_env.split(",") if origin.strip()]
else:
    allowed_origins = ["http://localhost:3000"]

# Add common deployment origins
default_origins = [
    "http://localhost:3000",
    "http://localhost:3001",
    "http://localhost:8080",
    "http://localhost:5173",  # Vite default
    "http://localhost:5174",  # Vite with proxy
    "https://127.0.0.1:3000",
    "https://127.0.0.1:8080",
    "https://localhost:3000",
    "https://localhost:8080",
]

# Add GitHub Pages origin if configured
if GITHUB_PAGES_URL:
    default_origins.append(GITHUB_PAGES_URL)

# Combine all origins, prioritizing environment variable
all_origins = list(set(allowed_origins + default_origins))

app.add_middleware(
    CORSMiddleware,
    allow_origins=all_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/health")
async def health_check():
    return {
        "status": "ok",
        "message": "RAG Chatbot API"
    }

@app.get("/db-status")
async def db_status():
    """Check if database and Qdrant are connected"""
    try:
        # Check database connection by attempting a simple query
        import psycopg2
        from config import NEON_DATABASE_URL
        conn = psycopg2.connect(NEON_DATABASE_URL)
        cursor = conn.cursor()
        cursor.execute("SELECT 1")
        db_connected = True
        cursor.close()
        conn.close()
    except Exception as e:
        db_connected = False
        db_error = str(e)

    # Check Qdrant connection
    try:
        qdrant_connected = hasattr(app.state, 'qdrant_store')
        if qdrant_connected:
            # Try to get collections to verify connection
            app.state.qdrant_store.client.get_collections()
    except Exception as e:
        qdrant_connected = False
        qdrant_error = str(e)

    return {
        "status": "ok",
        "database_connected": db_connected,
        "qdrant_connected": qdrant_connected,
        "message": "Database and Qdrant connection status"
    }


@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """Chat endpoint that processes user queries with context from book embeddings"""
    user_query = request.user_query
    user_id = request.user_id
    selected_text = request.selected_text

    # Search Qdrant for relevant chunks if no selected text
    if not selected_text:
        # First, generate an embedding for the user query using OpenAI
        from openai import OpenAI
        if not OPENAI_API_KEY:
            raise ValueError("OPENAI_API_KEY environment variable not set")

        client = OpenAI(api_key=OPENAI_API_KEY)
        query_embedding_response = client.embeddings.create(
            input=user_query,
            model="text-embedding-3-small"
        )
        query_embedding = query_embedding_response.data[0].embedding

        # Search Qdrant for top 3 most relevant chunks
        qdrant_store = app.state.qdrant_store
        search_results = qdrant_store.search(query_embedding, top_k=3)

        # Build context from search results
        context_parts = []
        sources = []

        for result in search_results:
            payload = result['payload']
            content = payload.get('content', '')
            source_file = payload.get('source_file', 'unknown')

            context_parts.append(content)
            sources.append({
                "source_file": source_file,
                "content": content[:200] + "..." if len(content) > 200 else content,  # Truncate for response
                "score": result['score']
            })

        # Combine the context
        context = "\n\n".join(context_parts)
    else:
        # Use selected text as context
        context = ""
        sources = [{"source_file": "selected_text", "content": selected_text}]

    # Use the RAG service to generate the answer with fallback options
    rag_service = app.state.rag_service
    response_data = rag_service.generate_answer(
        user_query=user_query,
        context=context,
        selected_text=selected_text
    )

    ai_answer = response_data["answer"]

    # Save the chat interaction to the database
    save_chat(user_id, user_query, ai_answer, str([source['source_file'] for source in sources]))

    # Return the response
    return ChatResponse(answer=ai_answer, sources=sources)


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)