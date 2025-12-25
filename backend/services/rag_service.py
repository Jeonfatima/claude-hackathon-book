# backend/services/rag_service.py
from services.retrieval_service import RetrievalService
from services.generation_service import GenerationService
from datetime import datetime
import uuid
import logging

logger = logging.getLogger(__name__)

class RAGService:
    def __init__(self):
        self.retrieval_service = RetrievalService()
        self.generation_service = GenerationService()

    async def process_query(self, query: str):
        # 1. Retrieve relevant chunks from vector DB
        retrieved_context = self.retrieval_service.retrieve(query)
        
        # 2. Extract texts (Ensure context is not empty to avoid model confusion)
        context_texts = getattr(retrieved_context, 'content_chunks', [])
        if not context_texts:
            logger.warning(f"No context found for query: {query}")
        
        # 3. Generate answer using the GenerationService (verified for 2025 Router)
        answer, confidence = self.generation_service.generate_response(query, context_texts)

        # 4. Format Sources for the Frontend
        sources = []
        meta_list = getattr(retrieved_context, 'source_metadata', [])
        score_list = getattr(retrieved_context, 'similarity_scores', [])
        
        # Zip safely even if lists are different lengths
        for meta, score in zip(meta_list, score_list):
            sources.append({
                "source_url": meta.get("url") or meta.get("source_url", "#"),
                "document_title": meta.get("title") or meta.get("document_title", "Unknown Document"),
                "excerpt": meta.get("text", meta.get("excerpt", ""))[:200], # First 200 chars
                "similarity_score": round(float(score), 4) if score else 0.0
            })

        # 5. Final JSON structure sent to Frontend
        response_payload = {
            "answer": answer,
            "sources": sources,
            "confidence_score": confidence,  # Changed from "confidence" to "confidence_score" to match frontend expectation
            "query_id": str(uuid.uuid4()),
            "response_timestamp": datetime.utcnow().isoformat() + "Z"
        }
        
        # IMPORTANT: Debug log to see what is leaving the backend
        logger.info(f"Final response prepared for {response_payload['query_id']}")
        
        print("==== RETRIEVED CONTEXT ====")
        for c in context_texts:
           print(c[:300])
        
        return response_payload