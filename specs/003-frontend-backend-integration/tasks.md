---
description: "Task list for frontend-backend integration chatbot feature"
---

# Tasks: Frontend-Backend Integration (Chatbot)

**Input**: Design documents from `/specs/003-frontend-backend-integration/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: No explicit tests requested in feature specification, so test tasks are omitted.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/`
- For Docusaurus frontend: `textbook/src/`
- For FastAPI backend: `backend/src/`

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create chatbot component directory structure in textbook/src/components/Chatbot/
- [x] T002 [P] Install required frontend dependencies for chat UI in textbook/package.json
- [x] T003 Configure environment variables for backend API URL in textbook/.env

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Verify backend CORS configuration allows frontend requests in backend/main.py
- [x] T005 [P] Create frontend ChatMessage model interface in textbook/src/types/chat.ts
- [x] T006 [P] Create frontend ChatSession model interface in textbook/src/types/chat.ts
- [x] T007 Create API service layer for backend communication in textbook/src/services/api.ts
- [x] T008 Configure API request/response validation based on contract in textbook/src/services/api.ts
- [x] T009 Set up frontend state management for chat functionality in textbook/src/contexts/ChatContext.tsx

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Chat with Book Assistant (Priority: P1) 🎯 MVP

**Goal**: Enable users to interact with a chatbot named "Fatima Salman" to ask questions about book content and get relevant answers

**Independent Test**: Can be fully tested by opening the chatbot UI, typing a question about the book, and receiving a response from "Fatima Salman". This delivers the core value of getting help with book content.

### Implementation for User Story 1

- [x] T010 [P] [US1] Create Chatbot UI component structure in textbook/src/components/Chatbot/Chatbot.tsx
- [x] T011 [P] [US1] Create Chatbot CSS styles in textbook/src/components/Chatbot/Chatbot.module.css
- [x] T012 [US1] Implement assistant name display as "Fatima Salman" in Chatbot component
- [x] T013 [US1] Add question input field and submission functionality in Chatbot component
- [x] T014 [US1] Implement API call to backend /query endpoint from Chatbot component
- [x] T015 [US1] Display assistant response in the chat UI with proper formatting
- [x] T016 [US1] Integrate Chatbot component with Docusaurus layout without breaking existing functionality

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Ask Questions About Selected Text (Priority: P2)

**Goal**: Enable users to select specific text on a book page and ask questions about only that text, making the assistant more precise and relevant

**Independent Test**: Can be fully tested by selecting text on a page, activating the chatbot with the selection, asking a question about it, and receiving a response focused on the selected content.

### Implementation for User Story 2

- [x] T017 [P] [US2] Implement text selection capture functionality in textbook/src/utils/textSelection.ts
- [x] T018 [US2] Add event listeners to capture user-selected text on book pages
- [x] T019 [US2] Store selected text in component state for context in Chatbot component
- [x] T020 [US2] Modify API call to include selected_text in request payload to backend
- [x] T021 [US2] Add UI element to initiate chat with selected text context in Chatbot component

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - View Assistant Responses in UI (Priority: P1)

**Goal**: Ensure users can clearly see the assistant's responses in the chat interface with distinguishable user vs assistant messages and proper loading/error states

**Independent Test**: Can be fully tested by sending a question to the assistant and observing that the response appears clearly in the chat UI with proper formatting and readability.

### Implementation for User Story 3

- [x] T022 [P] [US3] Implement loading state display during API requests in Chatbot component
- [x] T023 [P] [US3] Implement error handling and display for failed requests in Chatbot component
- [x] T024 [US3] Add visual distinction between user and assistant messages in chat UI
- [x] T025 [US3] Format assistant responses with proper styling and readability
- [x] T026 [US3] Implement message history display with timestamps in Chatbot component

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T027 [P] Add documentation for chatbot integration in textbook/docs/chatbot-integration.md
- [x] T028 Code cleanup and refactoring of chat components
- [x] T029 Performance optimization to ensure page load times don't exceed 10% increase
- [x] T030 [P] Accessibility improvements for chatbot UI components
- [x] T031 Security validation of user inputs before sending to backend
- [x] T032 Run quickstart.md validation to ensure all functionality works as expected

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Builds on US1 components
- **User Story 3 (P1)**: Can start after Foundational (Phase 2) - Builds on US1 components

### Within Each User Story

- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all components for User Story 1 together:
Task: "Create Chatbot UI component structure in textbook/src/components/Chatbot/Chatbot.tsx"
Task: "Create Chatbot CSS styles in textbook/src/components/Chatbot/Chatbot.module.css"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify functionality after each task or logical group
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence