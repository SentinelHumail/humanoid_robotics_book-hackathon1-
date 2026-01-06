import psycopg2
from psycopg2.extras import RealDictCursor
from config import NEON_DATABASE_URL
from typing import Optional, List, Dict, Any


def get_db_connection():
    """Function to connect to Neon Postgres using psycopg2"""
    return psycopg2.connect(NEON_DATABASE_URL)


def init_database():
    """Initialize the database by creating required tables if they don't exist"""
    conn = get_db_connection()
    cursor = conn.cursor()

    # Create chat_history table
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS chat_history (
            id SERIAL PRIMARY KEY,
            user_id VARCHAR(255),
            question TEXT,
            answer TEXT,
            sources TEXT,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        );
    """)

    # Create book_chunks table
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS book_chunks (
            id SERIAL PRIMARY KEY,
            content TEXT,
            source_file VARCHAR(500),
            chapter VARCHAR(255),
            chunk_index INTEGER,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        );
    """)

    conn.commit()
    cursor.close()
    conn.close()


def save_chat(user_id: str, question: str, answer: str, sources: Optional[str] = None):
    """Save a chat interaction to the database"""
    conn = get_db_connection()
    cursor = conn.cursor()

    cursor.execute("""
        INSERT INTO chat_history (user_id, question, answer, sources)
        VALUES (%s, %s, %s, %s)
    """, (user_id, question, answer, sources))

    conn.commit()
    cursor.close()
    conn.close()


def get_chat_history(user_id: str, limit: int = 10) -> List[Dict[str, Any]]:
    """Retrieve chat history for a specific user"""
    conn = get_db_connection()
    cursor = conn.cursor(cursor_factory=RealDictCursor)

    cursor.execute("""
        SELECT id, user_id, question, answer, sources, created_at
        FROM chat_history
        WHERE user_id = %s
        ORDER BY created_at DESC
        LIMIT %s
    """, (user_id, limit))

    rows = cursor.fetchall()
    cursor.close()
    conn.close()

    # Convert RealDictRow objects to dictionaries
    return [dict(row) for row in rows]