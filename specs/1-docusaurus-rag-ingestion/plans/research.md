# Research Findings: Docusaurus URL Ingestion System

**Feature**: Docusaurus URL ingestion, embedding generation, and vector storage
**Date**: 2025-12-14

## 1. Cohere API Integration

### Decision: Use Cohere v3 English Embedding Model
**Rationale**: Cohere's embed-english-v3.0 model provides high-quality embeddings for documentation text and has good performance for semantic search applications. It supports up to 5,120 tokens per request.

**Technical Details**:
- Model: `embed-english-v3.0`
- Input type: `search_document` for stored documents, `search_query` for queries
- Vector dimension: 1024
- Maximum tokens: 5,120 per request
- Rate limits: Vary by plan, typically 100-1000 RPM

**API Authentication**:
- Requires API key passed as `Authorization: Bearer <api_key>` header
- Best practice: Store in environment variable `COHERE_API_KEY`

### Alternatives Considered:
- `embed-multilingual-v3.0`: For multilingual content (not needed for this English-focused textbook)
- `embed-english-light-v3.0`: Smaller/faster but less accurate model
- Previous v2 models: Older, less capable than v3

## 2. Qdrant Setup

### Decision: Configure Qdrant Collection with 1024-dimensional vectors
**Rationale**: Cohere's embed-english-v3.0 produces 1024-dimensional vectors, so the Qdrant collection must match this dimension.

**Technical Details**:
- Vector size: 1024 (for Cohere embeddings)
- Distance function: Cosine (optimal for semantic similarity)
- Collection name: `rag_embeddings`
- Payload schema: Include content URL, title, and chunk metadata

**Configuration**:
```python
vector_params = VectorParams(
    size=1024,
    distance=Distance.COSINE
)
```

### Alternatives Considered:
- Different distance metrics (Euclidean, Dot): Cosine is standard for embeddings
- Different vector sizes: Must match embedding model output
- Multiple vector storages: Single collection approach is simpler

## 3. Docusaurus Content Extraction

### Decision: Use CSS selectors to target main content areas
**Rationale**: Docusaurus sites follow predictable patterns with main content in `.markdown` or `main` elements, allowing for consistent extraction.

**Technical Approach**:
- Target CSS selectors: `.markdown`, `.theme-doc-markdown`, `article`
- Exclude selectors: `nav`, `.navbar`, `.footer`, `.sidebar`, `.table-of-contents`
- Use BeautifulSoup to parse HTML and extract text
- Preserve heading hierarchy with custom logic

**Content Extraction Strategy**:
1. Parse HTML with BeautifulSoup
2. Identify main content container
3. Remove navigation, headers, footers
4. Extract text while preserving structure
5. Clean up excessive whitespace and formatting

### Alternatives Considered:
- Generic HTML cleaning libraries: Less precise for Docusaurus sites
- Browser automation (Selenium): More complex, slower
- Custom scraping rules per site: Less reusable

## 4. Text Chunking Strategy

### Decision: Use 1000-token chunks with 200-token overlap
**Rationale**: Balances context preservation with API limits, allowing for semantic coherence while fitting within Cohere's token constraints.

**Technical Details**:
- Chunk size: 1000 tokens (roughly 750-800 words)
- Overlap: 200 tokens (preserves context across chunks)
- Sentence boundary awareness: Don't break sentences mid-chunk
- Minimum chunk size: 100 tokens to ensure meaningful content

**Chunking Algorithm**:
1. Split text by paragraphs first
2. Combine paragraphs until reaching chunk size
3. Add overlap by including some content from previous chunk
4. Ensure semantic coherence

### Alternatives Considered:
- Fixed character counts: Less semantically meaningful
- Sentence-level chunks: Too granular, might lose context
- Larger chunks: Risk hitting token limits, less precise retrieval
- No overlap: Context loss at chunk boundaries

## 5. URL Discovery Strategy

### Decision: Implement breadth-first crawling with URL filtering
**Rationale**: Ensures comprehensive coverage of the Docusaurus site while avoiding external links or duplicate content.

**Technical Approach**:
- Start with base URL
- Extract all internal links from pages
- Filter for same-domain URLs
- Track visited URLs to avoid duplicates
- Respect robots.txt and implement rate limiting

## 6. Error Handling Strategy

### Decision: Implement comprehensive error handling with retry logic
**Rationale**: External APIs and network requests are prone to failures that need to be handled gracefully.

**Error Handling Components**:
- API rate limit handling with exponential backoff
- Network timeout management
- Invalid content filtering
- Graceful degradation when parts of pipeline fail