import os
import logging
from pathlib import Path
from typing import List, Dict, Any
import re

from openai import OpenAI
from config import OPENAI_API_KEY
from vector_store import QdrantStore


# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


def extract_title_from_content(content: str) -> str:
    """Extract the title from markdown content (first H1 heading)"""
    # Look for the first H1 heading in the content
    match = re.search(r'^#\s+(.+)$', content, re.MULTILINE)
    if match:
        return match.group(1).strip()
    return "Untitled"


def clean_markdown_content(content: str) -> str:
    """Clean markdown content by removing frontmatter and other unwanted parts"""
    # Remove frontmatter (YAML between ---)
    content = re.sub(r'^---\n.*?\n---\n', '', content, flags=re.DOTALL)

    # Remove other common markdown elements that might not be relevant for embeddings
    # Remove image references
    content = re.sub(r'!\[.*?\]\(.*?\)', '', content)
    # Remove links but keep the text
    content = re.sub(r'\[(.*?)\]\(.*?\)', r'\1', content)

    return content.strip()


def chunk_markdown_content(content: str, chunk_size: int = 1000, overlap: int = 100) -> List[str]:
    """Chunk markdown content into smaller pieces"""
    cleaned_content = clean_markdown_content(content)

    # Split content by headers to keep related content together
    header_split = re.split(r'\n#{1,3}\s', cleaned_content)

    chunks = []
    current_chunk = ""

    for part in header_split:
        if len(part.strip()) == 0:
            continue

        if len(current_chunk) + len(part) < chunk_size:
            current_chunk += f"\n{part}"
        else:
            if current_chunk:
                chunks.append(current_chunk.strip())

            # If the part is larger than chunk_size, split it by paragraphs
            if len(part) > chunk_size:
                paragraphs = part.split('\n\n')
                temp_chunk = ""

                for para in paragraphs:
                    if len(temp_chunk) + len(para) < chunk_size:
                        temp_chunk += f"\n\n{para}"
                    else:
                        if temp_chunk:
                            chunks.append(temp_chunk.strip())
                            temp_chunk = para
                        else:
                            # If a single paragraph is too long, split by sentences
                            if len(para) > chunk_size:
                                sentences = re.split(r'[.!?]+\s+', para)
                                sentence_chunk = ""
                                for sentence in sentences:
                                    if len(sentence_chunk) + len(sentence) < chunk_size:
                                        sentence_chunk += f" {sentence}"
                                    else:
                                        if sentence_chunk:
                                            chunks.append(sentence_chunk.strip())
                                        sentence_chunk = sentence
                                if sentence_chunk:
                                    chunks.append(sentence_chunk.strip())
                            else:
                                temp_chunk = para

                if temp_chunk:
                    current_chunk = temp_chunk
            else:
                current_chunk = part

    if current_chunk:
        chunks.append(current_chunk.strip())

    # Add overlap between chunks if needed
    if overlap > 0 and len(chunks) > 1:
        overlapped_chunks = []
        for i, chunk in enumerate(chunks):
            if i > 0:
                # Add overlap from the previous chunk
                prev_end = chunks[i-1][-overlap:]
                chunk = prev_end + " ... " + chunk
            overlapped_chunks.append(chunk)
        return overlapped_chunks

    return chunks


def get_embeddings(texts: List[str]) -> List[List[float]]:
    """Generate embeddings for a list of texts using OpenAI API"""
    if not OPENAI_API_KEY:
        raise ValueError("OPENAI_API_KEY environment variable not set")

    client = OpenAI(api_key=OPENAI_API_KEY)

    # Prepare the request
    response = client.embeddings.create(
        input=texts,
        model="text-embedding-3-small"
    )

    # Extract embeddings from response
    embeddings = []
    for data in response.data:
        embeddings.append(data.embedding)

    return embeddings


def main():
    """Main function to process markdown files and store embeddings in Qdrant"""
    # Initialize Qdrant store
    qdrant_store = QdrantStore()

    # Find all markdown files in docs directory
    docs_path = Path("../../docs")  # Relative to the script location
    if not docs_path.exists():
        # Try absolute path from project root
        docs_path = Path(__file__).parent.parent / "docs"

    if not docs_path.exists():
        logger.error(f"Docs directory not found at {docs_path}")
        return

    md_files = list(docs_path.rglob("*.md")) + list(docs_path.rglob("*.mdx"))
    logger.info(f"Found {len(md_files)} markdown files to process")

    # Process each file
    all_vectors = []
    all_metadata = []

    for i, file_path in enumerate(md_files, 1):
        logger.info(f"Processing file {i}/{len(md_files)}: {file_path}")

        try:
            # Read file content
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract title from content
            title = extract_title_from_content(content)

            # Chunk the content
            chunks = chunk_markdown_content(content)

            logger.info(f"  - Split into {len(chunks)} chunks")

            # Process each chunk
            for j, chunk in enumerate(chunks):
                if len(chunk.strip()) == 0:
                    continue

                # Generate embedding for this chunk
                embedding = get_embeddings([chunk])[0]  # Get first (and only) embedding

                # Prepare metadata
                metadata = {
                    "source_file": str(file_path.relative_to(docs_path.parent)),
                    "title": title,
                    "content": chunk,
                    "chunk_index": j,
                    "file_path": str(file_path)
                }

                # Store for batch processing
                all_vectors.append(embedding)
                all_metadata.append(metadata)

        except Exception as e:
            logger.error(f"Error processing file {file_path}: {str(e)}")
            continue

    # Batch insert all embeddings into Qdrant
    if all_vectors:
        logger.info(f"Uploading {len(all_vectors)} embeddings to Qdrant...")

        # Process in batches to avoid memory issues
        batch_size = 100
        for i in range(0, len(all_vectors), batch_size):
            batch_vectors = all_vectors[i:i+batch_size]
            batch_metadata = all_metadata[i:i+batch_size]

            try:
                qdrant_store.add_vectors(batch_vectors, batch_metadata)
                logger.info(f"  Uploaded batch {i//batch_size + 1}/{(len(all_vectors)-1)//batch_size + 1}")
            except Exception as e:
                logger.error(f"Error uploading batch {i//batch_size + 1}: {str(e)}")
                continue

        logger.info(f"Successfully indexed {len(all_vectors)} chunks from {len(md_files)} files")
    else:
        logger.warning("No content was processed - no embeddings to upload")


if __name__ == "__main__":
    main()