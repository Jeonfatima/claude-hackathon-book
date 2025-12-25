# backend/chatbot_cli.py

from services.retrieval_service import RetrievalService
from services.generation_service import GenerationService

def run_chatbot():
    print("📚 Chatbot is ready! Type 'exit' to quit.")
    retriever = RetrievalService()
    generator = GenerationService()

    while True:
        query = input("\nYou: ")
        if query.lower() in ("exit", "quit"):
            print("👋 Goodbye!")
            break

        try:
            # Retrieve context from Qdrant
            context_obj = retriever.retrieve(query)

            # Convert RetrievedContext to List[Dict] with 'content' key
            context_for_gen = [
                {"content": chunk} for chunk in context_obj.content_chunks
            ] if context_obj.content_chunks else []

            # Print retrieved context for verification
            print("\n📖 Retrieved context from book:")
            print("\n".join(chunk["content"] for chunk in context_for_gen) if context_for_gen else "No relevant content found.")

            # Generate answer using Google Gemini
            answer, confidence = generator.generate_response(query, context_for_gen)

            # Print the chatbot answer
            print(f"\n🤖 Chatbot (confidence {confidence:.2f}):\n{answer}")

        except Exception as e:
            print(f"⚠️ An error occurred: {e}")

if __name__ == "__main__":
    run_chatbot()
