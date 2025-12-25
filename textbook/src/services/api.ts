import { QueryRequest, QueryResponse } from '../types/chat';

// Base API service class for backend communication
class ApiService {
  private baseUrl: string;
  private defaultHeaders: HeadersInit;

 constructor() {
    // 1. Set the URL to your deployed Hugging Face backend
    this.baseUrl = 'https://fatima05-rag-chatbot.hf.space';

    // 2. Use the API key from your backend
    const apiKey = 'test-api-key'; 

    // 3. Set headers
    this.defaultHeaders = {
      'Content-Type': 'application/json',
    };

    // 4. Attach the key
    if (apiKey) {
      this.defaultHeaders['Authorization'] = `Bearer ${apiKey.trim()}`;
      console.log("Auth header attached"); // This helps us debug in F12 Console
    }
  }

  // Sanitize input to prevent XSS and other injection attacks
  private sanitizeInput(input: string): string {
    if (typeof input !== 'string') {
      return '';
    }

    // Remove potentially dangerous characters/sequences
    return input
      .replace(/<script\b[^<]*(?:(?!<\/script>)<[^<]*)*<\/script>/gi, '') // Remove script tags
      .replace(/javascript:/gi, '') // Remove javascript: protocol
      .replace(/vbscript:/gi, '') // Remove vbscript: protocol
      .replace(/on\w+="[^"]*"/gi, '') // Remove event handlers
      .replace(/<iframe\b[^<]*(?:(?!<\/iframe>)<[^<]*)*<\/iframe>/gi, '') // Remove iframe tags
      .replace(/<object\b[^<]*(?:(?!<\/object>)<[^<]*)*<\/object>/gi, '') // Remove object tags
      .replace(/<embed\b[^<]*(?:(?!<\/embed>)<[^<]*)*<\/embed>/gi, '') // Remove embed tags
      .replace(/eval\(/gi, '') // Remove eval function calls
      .replace(/expression\(/gi, '') // Remove expression function calls
      .trim();
  }

  // Validate query request before sending
  private validateQueryRequest(request: QueryRequest): void {
    // Sanitize inputs first
    const sanitizedQuestion = this.sanitizeInput(request.question || '');
    const sanitizedSelectedText = this.sanitizeInput(request.selected_text || '');

    // Validate question
    if (!sanitizedQuestion) {
      throw new Error('Question is required and must not be empty after sanitization');
    }

    if (typeof request.question !== 'string') {
      throw new Error('Question must be a string');
    }

    if (sanitizedQuestion.length < 1 || sanitizedQuestion.length > 1000) {
      throw new Error('Question must be between 1 and 1000 characters after sanitization');
    }

    // Validate selected text if provided
    if (sanitizedSelectedText && typeof request.selected_text !== 'string') {
      throw new Error('Selected text must be a string if provided');
    }

    if (sanitizedSelectedText && (sanitizedSelectedText.length < 0 || sanitizedSelectedText.length > 2000)) {
      throw new Error('Selected text must be between 0 and 2000 characters after sanitization');
    }

    // Validate that at least one of question or selected_text is provided (after sanitization)
    if (!sanitizedQuestion && !sanitizedSelectedText) {
      throw new Error('Either question or selected_text must be provided after sanitization');
    }

    // Validate session ID if provided
    if (request.sessionId && typeof request.sessionId !== 'string') {
      throw new Error('Session ID must be a string if provided');
    }
  }

  // Validate query response after receiving
  private validateQueryResponse(response: any): QueryResponse {
    if (!response) {
      throw new Error('Response is null or undefined');
    }

    // Validate required fields
    if (!response.answer || typeof response.answer !== 'string') {
      throw new Error('Response must contain a valid answer string');
    }

    // Validate answer length - but be more lenient for shorter answers
    if (response.answer.length < 5 || response.answer.length > 5000) {
      throw new Error('Answer must be between 5 and 5000 characters');
    }

    if (!response.sources || !Array.isArray(response.sources)) {
      // Be more lenient - sources can be empty array if no sources are found
      response.sources = []; // Default to empty array if not provided
    }

    // Validate sources if they exist
    if (response.sources && Array.isArray(response.sources)) {
      response.sources.forEach((source: any, index: number) => {
        if (!source.source_url || typeof source.source_url !== 'string') {
          source.source_url = '#'; // Default to placeholder
        }

        if (!source.document_title || typeof source.document_title !== 'string') {
          source.document_title = 'Reference';
        }

        if (!source.excerpt || typeof source.excerpt !== 'string') {
          source.excerpt = 'No excerpt available';
        }

        if (typeof source.similarity_score !== 'number' ||
            source.similarity_score < 0 ||
            source.similarity_score > 1) {
          source.similarity_score = 0.5; // Default to 0.5 if invalid
        }
      });
    }

    // Validate other required fields
    if (!response.query_id || typeof response.query_id !== 'string') {
      // Generate temporary ID if not provided
      response.query_id = `temp-${Date.now()}`;
    }

    if (!response.response_timestamp || typeof response.response_timestamp !== 'string') {
      // Use current time if not provided
      response.response_timestamp = new Date().toISOString();
    }

    // Validate confidence score if provided
    if (response.confidence_score !== undefined) {
      if (typeof response.confidence_score !== 'number' ||
          response.confidence_score < 0 ||
          response.confidence_score > 1) {
        response.confidence_score = 0.5; // Default to 0.5 if invalid
      }
    }

    return response as QueryResponse;
  }

  // Method to send query to backend
  async query(request: QueryRequest): Promise<QueryResponse> {
    // Validate the request before sending
    this.validateQueryRequest(request);

    try {
      // Create the request payload
      const payload: QueryRequest = {
        question: request.question,
        selected_text: request.selected_text,
        sessionId: request.sessionId
      };

      console.log('Sending request to backend:', payload); // Debug log

      // Make the API call
      const response = await fetch(`${this.baseUrl}/query`, {
        method: 'POST',
        headers: this.defaultHeaders,
        body: JSON.stringify(payload)
      });

      // Check if the response status is OK
      if (!response.ok) {
        const errorText = await response.text();
        let errorMessage = `HTTP error! status: ${response.status}`;

        // Try to parse error details if possible
        try {
          const errorJson = JSON.parse(errorText);
          errorMessage = errorJson.detail || errorJson.message || errorMessage;
        } catch {
          // If parsing fails, use the raw error text
          errorMessage = errorText || errorMessage;
        }

        throw new Error(errorMessage);
      }

      // Parse the response
      const responseData = await response.json();
      console.log('Received response from backend:', responseData); // Debug log

      // Validate the response after receiving
      const validatedResponse = this.validateQueryResponse(responseData);
      console.log('Validated response:', validatedResponse); // Debug log

      return validatedResponse;
    } catch (error) {
      console.error('API call failed:', error); // Debug log
      // Re-throw the error with additional context
      if (error instanceof Error) {
        throw new Error(`API call failed: ${error.message}`);
      } else {
        throw new Error('API call failed with unknown error');
      }
    }
  }

  // Health check method
  async healthCheck(): Promise<boolean> {
    try {
      const response = await fetch(`${this.baseUrl}/health`, {
        method: 'GET',
        headers: this.defaultHeaders
      });

      return response.ok;
    } catch (error) {
      console.error('Health check failed:', error);
      return false;
    }
  }
}

// Export a singleton instance of the API service
export const apiService = new ApiService();

// Export the class itself for potential extension or testing
export default ApiService;