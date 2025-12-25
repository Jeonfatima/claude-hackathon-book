# Implementation Plan: Docusaurus URL Ingestion System

**Feature**: Docusaurus URL ingestion, embedding generation, and vector storage
**Branch**: 1-docusaurus-rag-ingestion
**Created**: 2025-12-14
**Status**: Draft

## Technical Context

This implementation will create a Python-based system that:
- Fetches text content from deployed Docusaurus websites
- Generates embeddings using the Cohere API
- Stores embeddings in Qdrant vector database
- Is contained in a single main.py file as requested

**Technologies to be used:**
- Python 3.x
- Cohere Python client
- Qdrant Python client
- BeautifulSoup4 (for HTML parsing)
- Requests (for HTTP operations)
- Uvicorn (for package management)

**Unknowns requiring research:**
- Cohere API authentication and rate limits
- Qdrant setup and collection configuration
- Docusaurus site structure and content extraction patterns
- Text chunking strategies for optimal embeddings

## Constitution Check

Based on the project constitution, this implementation must:
- ✅ Follow modular architecture principles (though in a single file, functions will be modular)
- ✅ Implement test-first approach (tests will be added in subsequent phase)
- ✅ Consider performance requirements (efficient processing and storage)
- ✅ Implement security-first approach (secure API key handling)
- ✅ Be accessible and well-documented

## Gates

- [ ] Architecture aligns with constitution principles
- [ ] Security considerations addressed
- [ ] Performance requirements achievable
- [ ] Dependencies properly managed

---

## Phase 0: Research & Unknown Resolution

### Research Tasks

#### 0.1 Cohere API Integration Research
**Decision**: Determine the best Cohere embedding model for documentation text
**Rationale**: Need to select appropriate model for semantic similarity of textbook content
**Alternatives considered**: Different Cohere models (embed-english-v2.0, embed-english-v3.0, etc.)

#### 0.2 Qdrant Setup Research
**Decision**: Configure Qdrant collection with appropriate vector dimensions
**Rationale**: Cohere embeddings have specific dimensions that must match Qdrant collection
**Alternatives considered**: Different vector dimensions, distance metrics

#### 0.3 Docusaurus Content Extraction Research
**Decision**: Identify optimal approach to extract clean text from Docusaurus sites
**Rationale**: Need to extract meaningful content while excluding navigation, headers, footers
**Alternatives considered**: Different HTML parsing strategies, CSS selectors

#### 0.4 Text Chunking Strategy Research
**Decision**: Determine optimal chunk size for embedding generation
**Rationale**: Balance between context preservation and API token limits
**Alternatives considered**: Different chunk sizes, overlap strategies

---

## Phase 1: Data Model & API Design

### Data Model: data-model.md

#### DocusaurusContent Entity
- **url**: String - The source URL of the content
- **title**: String - The title of the page
- **content**: String - The extracted text content
- **metadata**: Dict - Additional metadata (headers, hierarchy, etc.)
- **created_at**: DateTime - When the content was extracted

#### EmbeddingVector Entity
- **id**: String - Unique identifier for the vector
- **content_id**: String - Reference to source content
- **vector**: List[float] - The embedding vector values
- **payload**: Dict - Metadata associated with the vector
- **created_at**: DateTime - When the embedding was generated

#### QdrantCollection Entity
- **name**: String - Name of the collection (rag_embeddings)
- **vector_size**: Integer - Dimension of vectors in collection
- **distance**: String - Distance metric (Cosine, Euclidean, etc.)

### API Contracts

#### Core Functions Design
1. `get_all_urls(base_url)` - Discover all accessible URLs from the Docusaurus site
2. `extract_text_from_url(url)` - Extract clean text content from a single URL
3. `chunk_text(text, chunk_size=1000)` - Split text into processable chunks
4. `embed(text_chunks)` - Generate embeddings for text chunks using Cohere
5. `create_collection(collection_name)` - Initialize Qdrant collection
6. `save_chunk_to_qdrant(embedding, content, metadata)` - Store embedding in Qdrant
7. `main()` - Execute the complete ingestion pipeline

---

## Phase 2: Implementation Approach

### 2.1 Backend Structure Setup
- Create `backend/` directory
- Initialize Python package with proper dependencies
- Set up configuration for Cohere and Qdrant clients

### 2.2 URL Discovery Module
- Use sitemap.xml from https://hackathon-claude-textbook.vercel.app/sitemap.xml to automatically discover all book pages and feed them into get_all_urls()
- Implement `get_all_urls()` function to crawl the Docusaurus site
- Handle relative/absolute URLs and avoid duplicate processing
- Respect robots.txt and implement rate limiting

### 2.3 Content Extraction Module
- Implement `extract_text_from_url()` using BeautifulSoup
- Extract only main content, exclude navigation, headers, footers
- Preserve document structure and hierarchy information

### 2.4 Text Processing Module
- Implement `chunk_text()` with appropriate size limits
- Handle sentence boundaries and maintain context
- Add overlap between chunks if needed for context preservation

### 2.5 Embedding Generation Module
- Set up Cohere client with proper authentication
- Implement `embed()` function to generate embeddings
- Handle rate limits and API errors gracefully

### 2.6 Vector Storage Module
- Set up Qdrant client with proper configuration
- Implement `create_collection()` with appropriate schema
- Implement `save_chunk_to_qdrant()` with metadata storage

### 2.7 Main Execution Module
- Implement `main()` function that orchestrates the complete pipeline
- Add error handling and logging
- Provide command-line interface for URL input

## Phase 3: Quality Assurance

### 3.1 Testing Strategy
- Unit tests for each function
- Integration tests for end-to-end pipeline
- Performance tests for large document sets

### 3.2 Security Considerations
- Secure handling of API keys (environment variables)
- Input validation for URLs
- Rate limiting to prevent abuse

### 3.3 Performance Optimization
- Batch processing for embeddings
- Efficient memory usage during text processing
- Connection pooling for API clients

## Implementation Timeline

1. **Week 1**: Backend setup, package initialization, client setup
2. **Week 2**: URL discovery and content extraction modules
3. **Week 3**: Text processing and embedding generation
4. **Week 4**: Vector storage and main execution module
5. **Week 5**: Integration, testing, and documentation

## Success Criteria

- [ ] Complete ingestion pipeline executes successfully
- [ ] Embeddings stored in Qdrant with proper metadata
- [ ] System handles the target website: https://hackathon-claude-textbook.vercel.app/
- [ ] All functions properly implemented as specified
- [ ] Code follows project constitution principles