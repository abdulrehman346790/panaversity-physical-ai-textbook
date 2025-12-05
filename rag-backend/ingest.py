import os
import glob
import markdown
from bs4 import BeautifulSoup
from qdrant_client import QdrantClient
from qdrant_client.http import models
from openai import AsyncOpenAI
from dotenv import load_dotenv

load_dotenv()

# Configuration
DOCS_DIR = "./docs"  # Changed from ../docusaurus/book/docs to local docs folder
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
COLLECTION_NAME = "textbook_content"

if not all([QDRANT_URL, QDRANT_API_KEY, GEMINI_API_KEY]):
    print("Error: Missing API keys in .env file")
    exit(1)

# Initialize Clients
qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
# Use Gemini via OpenAI Compatibility
openai_client = AsyncOpenAI(
    api_key=GEMINI_API_KEY,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

async def get_embedding(text):
    response = await openai_client.embeddings.create(
        input=text,
        model="text-embedding-004"
    )
    return response.data[0].embedding

def parse_markdown(file_path):
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read()
    
    # Convert Markdown to Text (simple stripping)
    html = markdown.markdown(content)
    text = "".join(BeautifulSoup(html, "html.parser").findAll(text=True))
    
    # Simple chunking by paragraphs (can be improved)
    chunks = [p.strip() for p in text.split('\n\n') if len(p.strip()) > 50]
    return chunks

async def ingest_docs():
    print("Starting ingestion...")
    
    # Recreate Collection to ensure correct vector size
    qdrant.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE),
    )
    print(f"Recreated collection: {COLLECTION_NAME}")

    files = glob.glob(os.path.join(DOCS_DIR, "**/*.md"), recursive=True)
    total_chunks = 0
    
    for file_path in files:
        filename = os.path.basename(file_path)
        print(f"Processing {filename}...")
        
        chunks = parse_markdown(file_path)
        points = []
        
        for i, chunk in enumerate(chunks):
            embedding = await get_embedding(chunk)  # Added await
            points.append(models.PointStruct(
                id=total_chunks + i,
                vector=embedding,
                payload={
                    "source": filename,
                    "content": chunk
                }
            ))
        
        if points:
            qdrant.upsert(
                collection_name=COLLECTION_NAME,
                points=points
            )
            total_chunks += len(points)
            print(f"Uploaded {len(points)} chunks from {filename}")

    print(f"Ingestion complete! Total chunks: {total_chunks}")

if __name__ == "__main__":
    import asyncio
    asyncio.run(ingest_docs())
