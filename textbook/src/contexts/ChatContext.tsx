import React, { createContext, useContext, useReducer, ReactNode } from 'react';
import { ChatState, ChatMessage } from '../types/chat';

// Define action types for the reducer
type ChatAction =
  | { type: 'ADD_MESSAGE'; payload: ChatMessage }
  | { type: 'SET_INPUT_TEXT'; payload: string }
  | { type: 'SET_LOADING'; payload: boolean }
  | { type: 'SET_ERROR'; payload?: string }
  | { type: 'SET_SELECTED_TEXT'; payload?: string }
  | { type: 'SET_OPEN'; payload: boolean }
  | { type: 'CLEAR_MESSAGES' }
  | { type: 'RESET_CHAT' };

// Initial state for the chat
const initialState: ChatState = {
  messages: [],
  inputText: '',
  isLoading: false,
  isOpen: false,
};

// Reducer function to handle state changes
const chatReducer = (state: ChatState, action: ChatAction): ChatState => {
  switch (action.type) {
    case 'ADD_MESSAGE':
      return {
        ...state,
        messages: [...state.messages, action.payload],
      };
    case 'SET_INPUT_TEXT':
      return {
        ...state,
        inputText: action.payload,
      };
    case 'SET_LOADING':
      return {
        ...state,
        isLoading: action.payload,
      };
    case 'SET_ERROR':
      return {
        ...state,
        error: action.payload,
      };
    case 'SET_SELECTED_TEXT':
      return {
        ...state,
        selectedText: action.payload,
      };
    case 'SET_OPEN':
      return {
        ...state,
        isOpen: action.payload,
      };
    case 'CLEAR_MESSAGES':
      return {
        ...state,
        messages: [],
      };
    case 'RESET_CHAT':
      return {
        ...initialState,
        isOpen: state.isOpen, // Preserve open state when resetting
      };
    default:
      return state;
  }
};

// Create the context
interface ChatContextType {
  state: ChatState;
  addMessage: (message: ChatMessage) => void;
  setInputText: (text: string) => void;
  setLoading: (loading: boolean) => void;
  setError: (error?: string) => void;
  setSelectedText: (text?: string) => void;
  setOpen: (open: boolean) => void;
  clearMessages: () => void;
  resetChat: () => void;
}

const ChatContext = createContext<ChatContextType | undefined>(undefined);

// Provider component
interface ChatProviderProps {
  children: ReactNode;
}

export const ChatProvider: React.FC<ChatProviderProps> = ({ children }) => {
  const [state, dispatch] = useReducer(chatReducer, initialState);

  const addMessage = (message: ChatMessage) => {
    dispatch({ type: 'ADD_MESSAGE', payload: message });
  };

  const setInputText = (text: string) => {
    dispatch({ type: 'SET_INPUT_TEXT', payload: text });
  };

  const setLoading = (loading: boolean) => {
    dispatch({ type: 'SET_LOADING', payload: loading });
  };

  const setError = (error?: string) => {
    dispatch({ type: 'SET_ERROR', payload: error });
  };

  const setSelectedText = (text?: string) => {
    dispatch({ type: 'SET_SELECTED_TEXT', payload: text });
  };

  const setOpen = (open: boolean) => {
    dispatch({ type: 'SET_OPEN', payload: open });
  };

  const clearMessages = () => {
    dispatch({ type: 'CLEAR_MESSAGES' });
  };

  const resetChat = () => {
    dispatch({ type: 'RESET_CHAT' });
  };

  return (
    <ChatContext.Provider
      value={{
        state,
        addMessage,
        setInputText,
        setLoading,
        setError,
        setSelectedText,
        setOpen,
        clearMessages,
        resetChat,
      }}
    >
      {children}
    </ChatContext.Provider>
  );
};

// Custom hook to use the chat context
export const useChat = (): ChatContextType => {
  const context = useContext(ChatContext);
  if (context === undefined) {
    throw new Error('useChat must be used within a ChatProvider');
  }
  return context;
};

export default ChatContext;