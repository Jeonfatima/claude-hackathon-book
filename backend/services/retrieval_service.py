from typing import List, Dict, Any, Optional
import logging
import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models
import cohere

from services.base_service import BaseService
from models.entities import RetrievedContext

load_dotenv()

class RetrievalService(BaseService):
    """
    Service for connecting to Qdrant Cloud and performing vector similarity search using Cohere embeddings
    """
    def __init__(self, collection_name: str = "rag_embeddings"):
        super().__init__()
        self.collection_name = collection_name

        # Initialize Cohere client
        cohere_api_key = os.getenv("COHERE_API_KEY")
        if not cohere_api_key:
            raise ValueError("COHERE_API_KEY environment variable is required")
        self.cohere_client = cohere.Client(cohere_api_key)

        # Initialize Qdrant client
        qdrant_api_key = os.getenv("QDRANT_API_KEY")
        qdrant_url = os.getenv("QDRANT_URL")

        if qdrant_api_key and qdrant_url:
            self.qdrant_client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key
            )
        elif qdrant_url:
            self.qdrant_client = QdrantClient(url=qdrant_url)
        else:
            raise ValueError("QDRANT_URL environment variable is required")

    def validate_config(self) -> bool:
        """
        Validate that the service has all required configuration
        """
        required_vars = ["COHERE_API_KEY", "QDRANT_URL"]
        missing_vars = [var for var in required_vars if not os.getenv(var)]

        if missing_vars:
            self.log_error(f"Missing required environment variables: {missing_vars}")
            return False

        return True

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
            self.log_error(f"Error generating query embedding: {str(e)}")
            raise e

    def retrieve(self, query: str, top_k: int = 3) -> RetrievedContext:
        """
        Retrieve relevant content chunks from Qdrant based on the query
        """
        from datetime import datetime

        try:
            # Generate embedding for the query
            query_embedding = self.embed_query(query)

            # Retrieve from Qdrant
            search_result = self.qdrant_client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=top_k,
                with_payload=True
            )

            content_chunks = []
            source_metadata = []
            similarity_scores = []

            for point in search_result.points:
                content_chunks.append(point.payload.get("text", "") if point.payload else "")
                similarity_scores.append(point.score if hasattr(point, 'score') else 0)

                metadata = {
                    "source_url": point.payload.get("url", "") if point.payload else "",
                    "document_title": point.payload.get("title", "") if point.payload else "",
                    "chunk_index": point.payload.get("chunk_index", 0) if point.payload else 0,
                    "chunk_id": str(point.id) if hasattr(point, 'id') else ""
                }
                source_metadata.append(metadata)

            # If no content was retrieved, return an empty context
            if not content_chunks:
                self.log_warning(f"No relevant content found for query: {query[:50]}...")
                retrieved_context = RetrievedContext(
                    content_chunks=[],
                    source_metadata=[],
                    similarity_scores=[],
                    retrieval_timestamp=datetime.utcnow().isoformat() + "Z"
                )
            else:
                # Create RetrievedContext entity with actual timestamp
                retrieved_context = RetrievedContext(
                    content_chunks=content_chunks,
                    source_metadata=source_metadata,
                    similarity_scores=similarity_scores,
                    retrieval_timestamp=datetime.utcnow().isoformat() + "Z"
                )

            self.log_info(f"Retrieved {len(content_chunks)} chunks from Qdrant")
            return retrieved_context

        except Exception as e:
            self.log_error(f"Error retrieving from Qdrant: {str(e)}")
            raise e

    def check_connection(self) -> bool:
        """
        Check if the service can connect to Qdrant
        """
        try:
            # Try to get collection info to verify connection
            collection_info = self.qdrant_client.get_collection(self.collection_name)
            self.log_info(f"Successfully connected to Qdrant collection: {self.collection_name}")
            return True
        except Exception as e:
            self.log_error(f"Failed to connect to Qdrant: {str(e)}")
            return False