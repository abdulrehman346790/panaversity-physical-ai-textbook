import os
from agents import Agent, Runner, function_tool, OpenAIChatCompletionsModel, SQLiteSession
from agents.run import RunConfig
from openai import AsyncOpenAI
from qdrant_client import QdrantClient
from openai import OpenAI
from dotenv import load_dotenv

load_dotenv()

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
COLLECTION_NAME = "textbook_content"

qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
openai_client = AsyncOpenAI(api_key=GEMINI_API_KEY, base_url="https://generativelanguage.googleapis.com/v1beta/openai")
model = OpenAIChatCompletionsModel(
    model="gemini-2.5-flash",
    openai_client=openai_client,
    )
config = RunConfig(
    model=model,
    model_provider=openai_client, tracing_disabled=True
    )    

@function_tool
async def search_book_content(query: str) -> str:
    """
    Search the textbook for relevant content based on the user's query.
    Returns the most relevant text chunks.
    """
    try:
        print(f"[SEARCH] Searching for: {query}")
        
        # Generate embedding for query
        response = await openai_client.embeddings.create(
            input=query,
            model="text-embedding-004"
        )
        query_vector = response.data[0].embedding
        print(f"[SEARCH] Generated embedding vector of size: {len(query_vector)}")
        
        # Search Qdrant
        hits = qdrant.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_vector,
            limit=3
        )
        print(f"[SEARCH] Found {len(hits)} results from Qdrant")
        
        if not hits:
            return "No relevant content found in the textbook. The collection might be empty or the query doesn't match any documents."
        
        results = []
        for hit in hits:
            results.append(f"Source: {hit.payload['source']}\nContent: {hit.payload['content']}")
        
        return "\n\n".join(results)
    except Exception as e:
        error_msg = f"Error searching textbook: {str(e)}"
        print(f"[SEARCH ERROR] {error_msg}")
        import traceback
        traceback.print_exc()
        return error_msg

# 1. Book Expert Agent
book_expert_agent = Agent(
    name="Book Expert",
    instructions="You are an expert on Physical AI and Robotics. Answer questions strictly based on the provided textbook content. If the answer is not in the context, say you don't know.",
    tools=[search_book_content],
)

# 2. Triage Agent (Orchestrator)
triage_agent = Agent(
    name="Triage Agent",
    instructions="You are a helpful assistant for the Physical AI Textbook. If the user asks a technical question about the book, handoff to the Book Expert. For general greetings, reply politely.",
    handoffs=[book_expert_agent]
)

# Session management is handled in main.py's /chat endpoint
