import os
import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin, urlparse
import time
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv
import logging
from typing import List, Dict, Tuple, Optional
import re
import uuid
import xml.etree.ElementTree as ET
import json

# FastAPI imports
from fastapi import FastAPI, HTTPException, Depends, Request
from fastapi.middleware.cors import CORSMiddleware
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded
from datetime import datetime

# Import our new modules and services
from models.request_models import QueryRequest
from models.response_models import QueryResponse, HealthResponse, Source
from models.entities import RetrievedContext
from services.rag_service import RAGService
from services.retrieval_service import RetrievalService
from services.generation_service import GenerationService
from utils.error_handler import add_exception_handler
from utils.auth import api_key_auth, get_api_key_from_request

# Load environment variables
load_dotenv()

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Initialize rate limiter
limiter = Limiter(key_func=get_remote_address)
app = FastAPI(
    title="RAG Agent API",
    description="API for the Retrieval-Augmented Generation agent that answers questions based on book content",
    version="1.0.0"
)

# Add
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Configure appropriately for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Add rate limiting
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

# Add exception handling
add_exception_handler(app)

# Initialize the RAG service
rag_service = RAGService()

@app.get("/")
async def root():
    return {"message": "RAG Agent API is running"}

@app.post("/query", response_model=QueryResponse)
@limiter.limit("5/minute")  # Limit to 5 requests per minute per IP
async def query_endpoint(request: Request, query_request: QueryRequest, api_key: str = Depends(api_key_auth)):
    """Process a user query and return an answer based on retrieved content."""
    try:
        # Process the query through the RAG service
        result = await rag_service.process_query(query_request.question)
        return result
    except requests.exceptions.ConnectionError:
        logger.error("Connection error when accessing external services")
        raise HTTPException(status_code=503, detail="Service unavailable: connection error to external service")
    except Exception as e:
        logger.error(f"Error in query endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/health", response_model=HealthResponse)
async def health_check():
    """Health check endpoint."""
    try:
        # Check if we can connect to required services
        connection_status = rag_service.validate_connection()

        # Prepare detailed health information
        details = {
            "timestamp": datetime.utcnow().isoformat(),
            "services": {
                "retrieval_service": connection_status.get("retrieval_service", False),
                "generation_service": connection_status.get("generation_service", False),
            }
        }

        status = "healthy" if connection_status.get("overall_status", False) else "unhealthy"
        return HealthResponse(status=status, details=details)
    except Exception as e:
        logger.error(f"Health check failed: {str(e)}")
        return HealthResponse(status="unhealthy", details={"error": str(e), "timestamp": datetime.utcnow().isoformat()})

class DocusaurusIngestionSystem:
    def __init__(self):
        # Initialize Cohere client
        cohere_api_key = os.getenv("COHERE_API_KEY")
        if not cohere_api_key:
            raise ValueError("COHERE_API_KEY environment variable is required")
        self.cohere_client = cohere.Client(cohere_api_key)

        # Initialize Qdrant client
        qdrant_host = os.getenv("QDRANT_HOST", "localhost")
        qdrant_port = int(os.getenv("QDRANT_PORT", 6333))
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        # Check if host contains protocol (http:// or https://)
        if qdrant_host.startswith(('http://', 'https://')):
            if qdrant_api_key:
                self.qdrant_client = QdrantClient(
                    url=qdrant_host,
                    api_key=qdrant_api_key
                )
            else:
                self.qdrant_client = QdrantClient(url=qdrant_host)
        else:
            if qdrant_api_key:
                self.qdrant_client = QdrantClient(
                    host=qdrant_host,
                    port=qdrant_port,
                    api_key=qdrant_api_key
                )
            else:
                self.qdrant_client = QdrantClient(host=qdrant_host, port=qdrant_port)

    def get_urls_from_sitemap(self, base_url: str) -> List[str]:
        """
        Get URLs directly from sitemap.xml
        """
        sitemap_url = f"{base_url.rstrip('/')}/sitemap.xml"
        urls = []

        try:
            logger.info(f"Fetching sitemap from: {sitemap_url}")
            headers = {
                'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
            }
            response = requests.get(sitemap_url, timeout=15, headers=headers)
            response.raise_for_status()

            # Parse with ElementTree
            root = ET.fromstring(response.text)

            # Handle different sitemap namespaces
            namespaces = {
                'sitemap': 'http://www.sitemaps.org/schemas/sitemap/0.9',
                '': 'http://www.sitemaps.org/schemas/sitemap/0.9'
            }

            # Find all <url><loc> elements with namespace handling
            for url_elem in root.findall('.//{http://www.sitemaps.org/schemas/sitemap/0.9}url') or root.findall('.//url'):
                loc_elem = url_elem.find('{http://www.sitemaps.org/schemas/sitemap/0.9}loc') or url_elem.find('loc')
                if loc_elem is not None and loc_elem.text:
                    urls.append(loc_elem.text.strip())

            logger.info(f"Found {len(urls)} URLs from sitemap")
            return urls
        except Exception as e:
            logger.warning(f"Could not fetch or parse sitemap: {str(e)}")
            return []

    def get_all_urls(self, base_url: str) -> List[str]:
        """
        Discover all accessible URLs from the Docusaurus site using sitemap first, then crawling as fallback
        """
        logger.info(f"Starting URL discovery from: {base_url}")

        # First try to get URLs from sitemap
        sitemap_urls = self.get_urls_from_sitemap(base_url)
        if sitemap_urls:
            logger.info(f"Using {len(sitemap_urls)} URLs from sitemap")
            return sitemap_urls

        # If sitemap is not available or fails, fall back to crawling
        logger.info("Sitemap not available, falling back to crawling...")

        # Parse the base URL to get the domain
        parsed_base = urlparse(base_url)
        base_domain = f"{parsed_base.scheme}://{parsed_base.netloc}"

        # Set to store discovered URLs
        discovered_urls = set()
        to_visit = [base_url]
        visited = set()

        # Limit crawling to avoid taking too long
        max_urls = 20
        while to_visit and len(discovered_urls) < max_urls:
            current_url = to_visit.pop(0)

            # Skip if already visited
            if current_url in visited:
                continue

            visited.add(current_url)
            logger.info(f"Visiting: {current_url}")

            try:
                # Fetch the page
                headers = {
                    'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
                }
                response = requests.get(current_url, timeout=15, headers=headers)
                response.raise_for_status()

                # Add to discovered URLs
                discovered_urls.add(current_url)

                # Parse the HTML
                soup = BeautifulSoup(response.text, 'html.parser')

                # Find all links
                for link in soup.find_all('a', href=True):
                    href = link['href']

                    # Convert relative URLs to absolute
                    absolute_url = urljoin(current_url, href)

                    # Only add URLs from the same domain
                    if urlparse(absolute_url).netloc == parsed_base.netloc:
                        # Only add URLs that are likely to be content pages
                        if (absolute_url not in visited and
                            absolute_url not in to_visit and
                            not absolute_url.endswith(('.pdf', '.jpg', '.png', '.zip', '.exe', '.css', '.js', '.ico')) and
                            not '/tag/' in absolute_url and
                            not '/assets/' in absolute_url and
                            '#' not in absolute_url):  # Avoid anchor links
                            to_visit.append(absolute_url)

            except requests.exceptions.RequestException as e:
                logger.error(f"Network error fetching {current_url}: {str(e)}")
                continue
            except Exception as e:
                logger.error(f"Error processing {current_url}: {str(e)}")
                continue

            # Be respectful - add a small delay
            time.sleep(0.2)

        logger.info(f"Discovered {len(discovered_urls)} URLs through crawling")
        return list(discovered_urls)

    def extract_text_from_url(self, url: str) -> Tuple[str, str]:
        """
        Extract clean text content from a single URL with improved Docusaurus-specific selectors
        """
        logger.info(f"Extracting text from: {url}")

        try:
            headers = {
                'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
            }
            response = requests.get(url, timeout=15, headers=headers)
            response.raise_for_status()

            soup = BeautifulSoup(response.text, 'html.parser')

            # Remove unwanted elements
            for element in soup(['nav', 'header', 'footer', 'script', 'style', 'aside', 'meta', 'link', 'button', 'img']):
                element.decompose()

            # Try to find the main content area with multiple selectors for Docusaurus
            main_content = None
            selectors = [
                'main div[class*="markdown"]',
                'article',
                'div[class*="container"] div[class*="docItemContainer"]',
                'div[class*="theme-doc-markdown"]',
                'div[class*="markdown"]',
                'div[class*="docContent"]',
                'div[class*="main"]',
                'main'
            ]

            for selector in selectors:
                main_content = soup.select_one(selector)
                if main_content:
                    break

            # If still not found, use body or the whole document
            if not main_content:
                main_content = soup.find('body') or soup

            # Extract text
            text = main_content.get_text(separator=' ', strip=True)

            # Get title
            title_tag = soup.find('title')
            title = title_tag.get_text().strip() if title_tag else urlparse(url).path.split('/')[-1] or 'Untitled'

            # Clean up the text more thoroughly
            text = re.sub(r'\s+', ' ', text)  # Replace multiple whitespace with single space
            text = re.sub(r'\n+', '\n', text)  # Keep single newlines, remove excessive ones
            text = text.strip()

            logger.info(f"Extracted {len(text)} characters from {url}")
            return title, text

        except requests.exceptions.RequestException as e:
            logger.error(f"Network error extracting text from {url}: {str(e)}")
            return "Network Error", ""
        except Exception as e:
            logger.error(f"Error extracting text from {url}: {str(e)}")
            return "Error", ""

    def chunk_text(self, text: str, chunk_size: int = 800) -> List[str]:
        """
        Split text into processable chunks with improved logic
        """
        if not text:
            return []

        # First, try to split by paragraphs to keep related content together
        paragraphs = [p.strip() for p in text.split('\n\n') if p.strip()]

        chunks = []
        current_chunk = ""

        for paragraph in paragraphs:
            # Check if adding this paragraph would exceed chunk size
            if len(current_chunk) + len(paragraph) <= chunk_size:
                if current_chunk:
                    current_chunk += "\n\n" + paragraph
                else:
                    current_chunk = paragraph
            else:
                # If current chunk is not empty, save it
                if current_chunk:
                    chunks.append(current_chunk)

                # If paragraph is longer than chunk_size, split it
                if len(paragraph) > chunk_size:
                    # Split long paragraph into sentences
                    sentences = re.split(r'[.!?]+\s+', paragraph)
                    temp_chunk = ""

                    for sentence in sentences:
                        if len(temp_chunk) + len(sentence) <= chunk_size:
                            if temp_chunk:
                                temp_chunk += " " + sentence
                            else:
                                temp_chunk = sentence
                        else:
                            if temp_chunk:
                                chunks.append(temp_chunk)
                            temp_chunk = sentence

                    if temp_chunk:
                        current_chunk = temp_chunk
                else:
                    current_chunk = paragraph

        # Add the last chunk if it exists
        if current_chunk:
            chunks.append(current_chunk)

        # Filter out empty chunks
        chunks = [chunk for chunk in chunks if chunk.strip()]

        logger.info(f"Text chunked into {len(chunks)} chunks")
        return chunks

    def embed(self, text_chunks: List[str]) -> List[List[float]]:
        """
        Generate embeddings for text chunks using Cohere with improved error handling
        """
        if not text_chunks:
            logger.warning("No text chunks to embed")
            return []

        logger.info(f"Generating embeddings for {len(text_chunks)} text chunks")

        # Cohere has a limit on batch size, so we process in chunks if needed
        batch_size = 96  # Cohere's recommended batch size
        all_embeddings = []

        for i in range(0, len(text_chunks), batch_size):
            batch = text_chunks[i:i+batch_size]

            try:
                response = self.cohere_client.embed(
                    texts=batch,
                    model="embed-english-v3.0",
                    input_type="search_document"
                )

                batch_embeddings = [embedding for embedding in response.embeddings]
                all_embeddings.extend(batch_embeddings)

                logger.info(f"Processed batch {i//batch_size + 1}, got {len(batch_embeddings)} embeddings")

                # Be respectful to the API
                time.sleep(0.1)

            except Exception as e:
                logger.error(f"Error generating embeddings for batch {i//batch_size + 1}: {str(e)}")
                # In production, you might want to implement retry logic here
                raise e

        logger.info(f"Generated {len(all_embeddings)} total embeddings")
        return all_embeddings

    def create_collection(self, collection_name: str = "rag_embeddings"):
        """
        Initialize Qdrant collection
        """
        logger.info(f"Creating/verifying collection: {collection_name}")

        try:
            # Check if collection exists
            self.qdrant_client.get_collection(collection_name)
            logger.info(f"Collection {collection_name} already exists")
        except:
            # Create collection if it doesn't exist
            # Cohere embeddings are 1024-dimensional
            self.qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE)
            )
            logger.info(f"Created collection {collection_name}")

    def save_chunk_to_qdrant(self, embedding: List[float], content: str, metadata: Dict, collection_name: str = "rag_embeddings"):
        """
        Store embedding in Qdrant with metadata, using the required payload format
        """
        point_id = str(uuid.uuid4())

        try:
            self.qdrant_client.upsert(
                collection_name=collection_name,
                points=[
                    models.PointStruct(
                        id=point_id,
                        vector=embedding,
                        payload={
                            "url": metadata.get("url", ""),
                            "text": content,  # Using 'text' as specified in requirements
                            "title": metadata.get("title", ""),
                            "chunk_index": metadata.get("chunk_index", 0),
                            "created_at": time.time()
                        }
                    )
                ]
            )
            logger.info(f"Saved chunk to Qdrant with ID: {point_id}, URL: {metadata.get('url', 'unknown')}")
            return point_id
        except Exception as e:
            logger.error(f"Error saving chunk to Qdrant: {str(e)}")
            raise e

    def embed_query(self, query: str) -> List[float]:
        """
        Generate embedding for a query string using Cohere
        """
        try:
            response = self.cohere_client.embed(
                texts=[query],
                model="embed-english-v3.0",
                input_type="search_query"  # Using search_query for retrieval
            )
            return response.embeddings[0]
        except Exception as e:
            logger.error(f"Error generating query embedding: {str(e)}")
            raise e

    def retrieve_from_qdrant(self, query_embedding: List[float], top_k: int = 3, collection_name: str = "rag_embeddings") -> List[Dict]:
        """
        Retrieve top-k matches from Qdrant based on query embedding
        """
        try:
            # Use the query_points method for newer Qdrant client versions
            search_result = self.qdrant_client.query_points(
                collection_name=collection_name,
                query=query_embedding,
                limit=top_k,
                with_payload=True
            )

            results = []
            for point in search_result.points:
                result = {
                    "content": point.payload.get("text", "") if point.payload else "",
                    "similarity_score": point.score if hasattr(point, 'score') else 0,
                    "metadata": {
                        "url": point.payload.get("url", "") if point.payload else "",
                        "chunk_id": str(point.id) if hasattr(point, 'id') else "",
                        "title": point.payload.get("title", "") if point.payload else ""
                    }
                }
                results.append(result)

            logger.info(f"Retrieved {len(results)} chunks from Qdrant")
            return results
        except Exception as e:
            logger.error(f"Error retrieving from Qdrant with query_points method: {str(e)}")
            # Try the query method as fallback
            try:
                search_result = self.qdrant_client.query(
                    collection_name=collection_name,
                    query=query_embedding,
                    limit=top_k,
                    with_payload=True
                )

                results = []
                for point in search_result:
                    result = {
                        "content": point.payload.get("text", "") if point.payload else "",
                        "similarity_score": point.score if hasattr(point, 'score') else 0,
                        "metadata": {
                            "url": point.payload.get("url", "") if point.payload else "",
                            "chunk_id": str(point.id) if hasattr(point, 'id') else "",
                            "title": point.payload.get("title", "") if point.payload else ""
                        }
                    }
                    results.append(result)

                logger.info(f"Retrieved {len(results)} chunks from Qdrant using query method")
                return results
            except Exception as e2:
                logger.error(f"Both query methods failed: {str(e2)}")
                raise e

    def validate_content_accuracy(self, retrieved_chunks: List[Dict], expected_content: Optional[str] = None) -> float:
        """
        Validate that retrieved chunks match expected/original content
        For testing purposes, we'll check content similarity based on overlap
        """
        if not retrieved_chunks:
            return 0.0

        accurate_chunks = 0
        for chunk in retrieved_chunks:
            retrieved_content = chunk.get("content", "")
            # If we have expected content, validate against it
            if expected_content:
                # Simple overlap check - if retrieved content contains or is contained in expected content
                if expected_content.lower() in retrieved_content.lower() or retrieved_content.lower() in expected_content.lower():
                    chunk["validation_status"] = "pass"
                    accurate_chunks += 1
                else:
                    chunk["validation_status"] = "fail"
            else:
                # Without expected content, we can't validate accuracy, so mark as unknown
                chunk["validation_status"] = "unknown"

        accuracy_percentage = (accurate_chunks / len(retrieved_chunks)) * 100 if retrieved_chunks else 0.0
        logger.info(f"Content accuracy: {accuracy_percentage}% ({accurate_chunks}/{len(retrieved_chunks)} chunks accurate)")
        return accuracy_percentage

    def validate_metadata_correctness(self, retrieved_chunks: List[Dict]) -> float:
        """
        Validate that metadata (URL, chunk_id) is correctly retrieved
        """
        if not retrieved_chunks:
            return 0.0

        correct_metadata_chunks = 0
        for chunk in retrieved_chunks:
            metadata = chunk.get("metadata", {})
            url = metadata.get("url", "")
            chunk_id = metadata.get("chunk_id", "")

            # Check if required metadata fields are present and not empty
            if url and chunk_id:
                correct_metadata_chunks += 1
            else:
                chunk["metadata_validation_status"] = "fail"

        correctness_percentage = (correct_metadata_chunks / len(retrieved_chunks)) * 100 if retrieved_chunks else 0.0
        logger.info(f"Metadata correctness: {correctness_percentage}% ({correct_metadata_chunks}/{len(retrieved_chunks)} chunks with correct metadata)")
        return correctness_percentage

    def test_retrieval_pipeline(self, query: str, top_k: int = 3, expected_content: Optional[str] = None) -> Dict:
        """
        Main function to test the RAG retrieval pipeline
        """
        start_time = time.time()

        logger.info(f"Testing retrieval pipeline with query: '{query}'")

        # Step 1: Embed the query
        query_embedding = self.embed_query(query)

        # Step 2: Retrieve from Qdrant
        retrieved_chunks = self.retrieve_from_qdrant(query_embedding, top_k)

        # Step 3: Validate content accuracy
        content_accuracy = self.validate_content_accuracy(retrieved_chunks, expected_content)

        # Step 4: Validate metadata correctness
        metadata_correctness = self.validate_metadata_correctness(retrieved_chunks)

        # Calculate execution time
        execution_time = time.time() - start_time

        # Calculate top-k relevance score (average similarity score)
        if retrieved_chunks:
            avg_similarity = sum(chunk.get("similarity_score", 0) for chunk in retrieved_chunks) / len(retrieved_chunks)
            top_k_relevance = avg_similarity * 100  # Convert to percentage
        else:
            top_k_relevance = 0.0

        # Determine overall status
        overall_status = "pass" if content_accuracy >= 50 and metadata_correctness >= 50 else "fail"  # Simple threshold

        # Prepare validation details
        validation_details = []
        for i, chunk in enumerate(retrieved_chunks):
            detail = {
                "chunk_id": chunk.get("metadata", {}).get("chunk_id", f"chunk-{i}"),
                "content_match": chunk.get("validation_status") == "pass",
                "metadata_match": chunk.get("metadata_validation_status") != "fail",
                "similarity_score": chunk.get("similarity_score", 0)
            }
            validation_details.append(detail)

        # Prepare the final result
        result = {
            "query": query,
            "retrieved_chunks": [
                {
                    "content": chunk.get("content", "")[:500] + "..." if len(chunk.get("content", "")) > 500 else chunk.get("content", ""),  # Truncate long content
                    "similarity_score": chunk.get("similarity_score", 0),
                    "metadata": chunk.get("metadata", {}),
                    "validation_status": chunk.get("validation_status", "unknown")
                }
                for chunk in retrieved_chunks
            ],
            "execution_time": round(execution_time, 3),
            "validation_results": {
                "content_accuracy": round(content_accuracy, 2),
                "metadata_correctness": round(metadata_correctness, 2),
                "top_k_relevance": round(top_k_relevance, 2),
                "overall_status": overall_status,
                "details": validation_details
            }
        }

        logger.info(f"Retrieval test completed in {execution_time:.3f}s with {overall_status.upper()} status")
        return result

    def validate_connection(self) -> Dict:
        """
        Validate connection to Qdrant vector database
        """
        try:
            # Check if collection exists
            collection_info = self.qdrant_client.get_collection("rag_embeddings")
            connected = True
            collection_exists = True
            points_count = collection_info.points_count
            logger.info(f"Connected to Qdrant. Collection 'rag_embeddings' exists with {points_count} points.")
        except Exception as e:
            logger.error(f"Connection to Qdrant failed: {str(e)}")
            connected = False
            collection_exists = False
            points_count = 0

        return {
            "connected": connected,
            "collection_exists": collection_exists,
            "points_count": points_count
        }

def main():
    import sys
    import argparse

    # Parse command line arguments
    parser = argparse.ArgumentParser(description="RAG Retrieval Pipeline Testing")
    parser.add_argument("--mode", choices=["ingest", "test", "validate-connection"], default="test",
                       help="Mode to run: ingest (original ingestion), test (retrieval testing), or validate-connection")
    parser.add_argument("--query", type=str, help="Query to test retrieval with")
    parser.add_argument("--top-k", type=int, default=3, help="Number of top results to retrieve")
    parser.add_argument("--expected-content", type=str, help="Expected content for validation")
    parser.add_argument("--batch-test", action="store_true", help="Run comprehensive batch tests")

    args = parser.parse_args()

    logger.info("Starting RAG Retrieval Pipeline Testing")
    logger.info("Note: Ensure your .env file has valid COHERE_API_KEY and QDRANT credentials")

    # Initialize the system
    system = DocusaurusIngestionSystem()

    if args.mode == "validate-connection":
        # Validate connection to Qdrant
        connection_status = system.validate_connection()
        print(json.dumps(connection_status, indent=2))
        return connection_status

    elif args.mode == "ingest":
        # Original ingestion pipeline
        try:
            # Create collection
            system.create_collection("rag_embeddings")

            # Counters for progress tracking
            total_pages_processed = 0
            total_chunks_stored = 0
            total_urls_discovered = 0

            # Use the target Docusaurus site URL
            base_url = "https://hackathon-claude-textbook.vercel.app/"

            # Discover all URLs using sitemap first, then crawling as fallback
            urls = system.get_all_urls(base_url)
            total_urls_discovered = len(urls)
            logger.info(f"Starting to process {total_urls_discovered} URLs")

            # Process each URL
            for i, url in enumerate(urls):
                logger.info(f"Processing {i+1}/{len(urls)}: {url}")
                title, text = system.extract_text_from_url(url)

                if not text or len(text.strip()) < 50:  # Skip pages with very little content
                    logger.warning(f"Insufficient text ({len(text)} chars) extracted from {url}, skipping")
                    continue

                chunks = system.chunk_text(text)
                if not chunks:
                    logger.warning(f"No chunks created from {url}, skipping")
                    continue

                # Process each chunk
                for j, chunk in enumerate(chunks):
                    try:
                        embeddings = system.embed([chunk])
                        if embeddings and len(embeddings) > 0:
                            metadata = {"url": url, "title": title, "chunk_index": j}
                            point_id = system.save_chunk_to_qdrant(embeddings[0], chunk, metadata, "rag_embeddings")
                            total_chunks_stored += 1
                            logger.info(f"Successfully stored chunk {j+1} of {len(chunks)} from page {i+1} - Point ID: {point_id}")
                        else:
                            logger.warning(f"No embeddings generated for chunk {j} of {url}")
                    except Exception as e:
                        logger.error(f"Failed to process chunk {j} of {url}: {str(e)}")
                        continue  # Continue with next chunk

                total_pages_processed += 1

            # Final check - count points in collection
            try:
                collection_info = system.qdrant_client.get_collection("rag_embeddings")
                points_count = collection_info.points_count
                logger.info(f"Final verification: Collection 'rag_embeddings' now contains {points_count} points")
            except Exception as e:
                logger.error(f"Could not verify collection contents: {str(e)}")

            logger.info(f"Docusaurus ingestion pipeline completed successfully!")
            logger.info(f"Summary: {total_urls_discovered} URLs discovered, {total_pages_processed} pages processed, {total_chunks_stored} chunks stored")

            return total_chunks_stored

        except Exception as e:
            logger.error(f"Pipeline failed: {str(e)}")
            logger.error("Please check:")
            logger.error("- Network connectivity to Cohere and Qdrant APIs")
            logger.error("- Valid API keys in your .env file")
            logger.error("- Correct Qdrant configuration")
            raise e

    elif args.mode == "test":
        # RAG retrieval testing pipeline
        try:
            # Validate connection first
            connection_status = system.validate_connection()
            if not connection_status.get("connected", False):
                logger.error("Cannot connect to Qdrant. Please check your configuration.")
                return {"error": "Connection to Qdrant failed"}

            if not connection_status.get("collection_exists", False):
                logger.error("Collection 'rag_embeddings' does not exist. Please run ingestion first.")
                return {"error": "Collection does not exist"}

            if connection_status.get("points_count", 0) == 0:
                logger.error("Collection 'rag_embeddings' is empty. Please run ingestion first.")
                return {"error": "Collection is empty"}

            # Test retrieval pipeline
            if args.batch_test:
                # Run comprehensive tests with multiple queries
                test_queries = [
                    "What is ROS 2?",
                    "Explain robot operating system",
                    "How to install ROS",
                    "ROS tutorials",
                    "Differences between ROS 1 and ROS 2"
                ]

                all_results = []
                for query in test_queries:
                    logger.info(f"Running batch test for query: {query}")
                    result = system.test_retrieval_pipeline(query, args.top_k, args.expected_content)
                    all_results.append(result)
                    print(json.dumps(result, indent=2))
                    print("-" * 80)  # Separator between tests

                # Summary of all tests
                total_tests = len(all_results)
                passed_tests = sum(1 for result in all_results if result["validation_results"]["overall_status"] == "pass")
                logger.info(f"Batch test completed: {passed_tests}/{total_tests} tests passed")

                summary = {
                    "total_tests": total_tests,
                    "passed_tests": passed_tests,
                    "success_rate": round((passed_tests / total_tests) * 100, 2),
                    "all_results": all_results
                }

                print(json.dumps(summary, indent=2))
                return summary

            else:
                # Run single test with provided query or default
                query = args.query or "What is ROS 2?"
                result = system.test_retrieval_pipeline(query, args.top_k, args.expected_content)

                print(json.dumps(result, indent=2))
                return result

        except Exception as e:
            logger.error(f"Retrieval test failed: {str(e)}")
            logger.error("Please check:")
            logger.error("- Network connectivity to Cohere and Qdrant APIs")
            logger.error("- Valid API keys in your .env file")
            logger.error("- Correct Qdrant configuration")
            logger.error("- Collection 'rag_embeddings' exists and has data")
            raise e


if __name__ == "__main__":
    main()