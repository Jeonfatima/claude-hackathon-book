// Chat Message Entity
export interface ChatMessage {
  id: string;                    // Unique identifier for the message
  sender: 'user' | 'assistant'; // Either "user" or "assistant"
  content: string;              // The text content of the message
  timestamp: string;            // When the message was created (ISO date string)
  status?: 'sent' | 'delivered' | 'error'; // Message status (optional)
}

// Chat Session Entity
export interface ChatSession {
  sessionId: string;            // Unique identifier for the session
  messages: ChatMessage[];      // Array of ChatMessage entities
  createdAt: string;            // When the session was created (ISO date string)
  lastActiveAt: string;         // When the session was last active (ISO date string)
}

// Query Request Entity (for API communication)
export interface QueryRequest {
  question: string;             // The user's question to be answered (1-1000 characters)
  selected_text?: string;       // Optional text selected by user on the page (0-2000 characters)
  sessionId?: string;           // Optional session identifier
}

// Source Entity (from API response)
export interface Source {
  source_url: string;           // URL of the source document
  document_title: string;       // Title of the source document
  excerpt: string;              // Relevant excerpt from the source
  similarity_score: number;     // Relevance score of this source (0-1)
}

// Query Response Entity (from API)
export interface QueryResponse {
  answer: string;               // The generated answer based on retrieved content
  sources: Source[];            // List of sources used to generate the answer
  query_id: string;             // Unique identifier for the query
  response_timestamp: string;   // Timestamp when the response was generated (ISO date format)
  confidence_score?: number;    // Optional confidence level in the answer (0-1)
}

// Frontend State Entity
export interface ChatState {
  messages: ChatMessage[];      // Array of ChatMessage entities
  inputText: string;            // Current text in the input field (0-1000 characters)
  isLoading: boolean;           // Whether an API request is in progress
  error?: string;               // Error message if any (optional)
  selectedText?: string;        // Text currently selected on the page (optional)
  isOpen: boolean;              // Whether the chat widget is open
}