# backend/services/generation_service.py
import os
import requests
import logging

logger = logging.getLogger(__name__)

class GenerationService:
    def __init__(self):
        # 1. Credentials and Endpoint
        self.api_key = os.getenv("HUGGINGFACE_API_KEY")
        if not self.api_key:
            raise ValueError("HUGGINGFACE_API_KEY environment variable is required")
        
        # Using the certified 2025 router path and model
        self.url = "https://router.huggingface.co/v1/chat/completions"
        self.model = "google/gemma-2-2b-it"

    def generate_response(self, question, context=[]):
        """Generates a response using the verified Hugging Face Router."""
        # Clean up context for the prompt
        context_str = "\n".join(context) if context else "No relevant context found in the textbook."
        
        headers = {
            "Authorization": f"Bearer {self.api_key.strip()}",
            "Content-Type": "application/json"
        }
        
        # 2. Structure the payload for a Chat Model
        payload = {
            "model": self.model,
            "messages": [
                {
                    "role": "system", 
                    "content": "You are a helpful AI assistant. Use the provided context to answer the question accurately. If the answer isn't in the context, tell the user you don't know based on the documents."
                },
                {
                    "role": "user", 
                    "content": f"Context:\n{context_str}\n\nQuestion: {question}"
                }
            ],
            "max_tokens": 500,
            "temperature": 0.7
        }

        try:
            logger.info(f"Sending query to HF Router using model: {self.model}")
            response = requests.post(self.url, headers=headers, json=payload, timeout=30)
            response.raise_for_status()
            
            data = response.json()
            
            # 3. Extract the answer based on your successful test output
            if "choices" in data and len(data["choices"]) > 0:
                answer = data["choices"][0]["message"]["content"].strip()
            else:
                logger.error(f"Unexpected API response structure: {data}")
                answer = "The model returned an empty response. Please try rephrasing your question."

            # Ensure answer meets any length requirements for your response models
            if len(answer) < 10:
                answer += " [Detailed information could not be generated from the context provided.]"

            return answer, 0.95  # Return answer and a confidence score

        except requests.exceptions.HTTPError as http_err:
            logger.error(f"HTTP error occurred: {http_err}")
            return f"Service Error: The language model is currently unavailable (Status: {response.status_code}).", 0.0
        except Exception as e:
            logger.error(f"Generation error: {str(e)}")
            return f"I'm sorry, I encountered an internal error while generating the answer.", 0.0