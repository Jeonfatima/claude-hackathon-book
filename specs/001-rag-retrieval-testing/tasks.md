---
description: "Validation-only task list for RAG retrieval testing"
---

# Tasks: 001 – RAG Retrieval Testing (Validation Only)

## Scope

This spec validates that embeddings already stored in Qdrant
can be successfully retrieved using a query embedding.

This is **NOT** a production backend implementation.

---

## Constraints

DO NOT:
- create `backend/src/`
- refactor architecture
- create APIs, services, endpoints, or models
- add tests or test frameworks
- introduce performance, security, or polish layers

ONLY:
- update existing `backend/main.py`
- connect to Qdrant
- generate query embeddings
- retrieve top-k vectors
- print results for manual verification

---

## Tasks

- [x] Verify Qdrant connection and collection existence
- [x] Generate embeddings for input query
- [x] Retrieve top-k vectors from Qdrant
- [x] Display retrieved text, metadata, and similarity scores
- [x] Manually verify retrieval correctness
- [x] Run batch queries for confidence check

---

## Completion Criteria

- Script runs via `python backend/main.py`
- Retrieval returns relevant chunks
- Metadata (id, url, title) is visible
- No new folders or modules added
- Spec goals satisfied without architectural changes
