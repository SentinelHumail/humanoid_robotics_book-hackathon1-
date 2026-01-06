from qdrant_client import QdrantClient
from qdrant_client.http import models
from config import QDRANT_URL, QDRANT_API_KEY
from typing import List, Dict, Any, Optional


class QdrantStore:
    """Class to handle Qdrant vector storage operations"""

    def __init__(self):
        """Initialize connection to Qdrant using QDRANT_URL and QDRANT_API_KEY"""
        try:
            if QDRANT_API_KEY:
                self.client = QdrantClient(
                    url=QDRANT_URL,
                    api_key=QDRANT_API_KEY,
                    prefer_grpc=False  # Using HTTP for better compatibility
                )
            else:
                self.client = QdrantClient(url=QDRANT_URL)
        except Exception as e:
            print(f"Error connecting to Qdrant: {e}")
            raise

    def init_collection(self):
        """Create 'book_embeddings' collection with vector size 1536 and Cosine distance"""
        try:
            # Check if collection already exists
            collections = self.client.get_collections()
            collection_names = [collection.name for collection in collections.collections]

            if "book_embeddings" not in collection_names:
                self.client.create_collection(
                    collection_name="book_embeddings",
                    vectors_config=models.VectorParams(
                        size=1536,  # Standard size for OpenAI embeddings
                        distance=models.Distance.COSINE
                    )
                )
            else:
                print("Collection 'book_embeddings' already exists")
        except Exception as e:
            print(f"Error initializing collection: {e}")
            raise

    def add_vectors(self, vectors: List[List[float]], metadata: List[Dict[str, Any]]):
        """Store embeddings with metadata"""
        try:
            # Prepare points for insertion
            points = []
            for i, (vector, meta) in enumerate(zip(vectors, metadata)):
                point = models.PointStruct(
                    id=i,  # This would need to be more sophisticated in production
                    vector=vector,
                    payload=meta
                )
                points.append(point)

            # Upsert the points
            self.client.upsert(
                collection_name="book_embeddings",
                points=points
            )
        except Exception as e:
            print(f"Error adding vectors: {e}")
            raise

    def search(self, query_vector: List[float], top_k: int = 5) -> List[Dict[str, Any]]:
        """Search for similar vectors"""
        try:
            results = self.client.search(
                collection_name="book_embeddings",
                query_vector=query_vector,
                limit=top_k
            )

            # Format results to return metadata
            formatted_results = []
            for result in results:
                formatted_results.append({
                    'id': result.id,
                    'score': result.score,
                    'payload': result.payload
                })

            return formatted_results
        except Exception as e:
            print(f"Error searching vectors: {e}")
            raise