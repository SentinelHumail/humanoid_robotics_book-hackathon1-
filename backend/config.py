from dotenv import load_dotenv
import os

# Load environment variables from .env file
load_dotenv()

# Export environment variables as constants
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
NEON_DATABASE_URL = os.getenv("NEON_DATABASE_URL")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
GITHUB_PAGES_URL = os.getenv("GITHUB_PAGES_URL", "")
ALLOWED_ORIGINS = os.getenv("ALLOWED_ORIGINS", "")