---
id: 1
title: "create textbook constitution"
stage: constitution
date_iso: "2025-12-07"
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: "$ARGUMENTS"
labels: ["constitution", "textbook", "physical-ai", "humanoid-robotics"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - .specify/memory/constitution.md
tests_yaml: |
  - none
outcome: |
  Successfully created comprehensive constitution for the Physical AI & Humanoid Robotics textbook project
evaluation: |
  The constitution includes all required elements: Purpose, Vision & Audience, Performance Goals, Non-Goals, Success Criteria, Features & Capabilities, Constraints, Scope, Deliverables, Risks, Assumptions, Architecture Overview, Document tree, and Expected folders.
---

# Prompt

```text
Project Title: "Physical AI & Humanoid Robotics – A Complete Textbook"

Overall Goal:
Write and publish a full Docusaurus-based textbook that teaches the official
"Physical AI & Humanoid Robotics" course. Also build and embed a RAG chatbot
inside the book using OpenAI Agents, FastAPI, Neon Postgres, and Qdrant Cloud.

Include support for:
- reusable intelligence via Claude Code Subagents
- personalized learning (Better-Auth)
- Urdu translation toggle
- chapter-level personalization button
- deploy the book to GitHub Pages or Vercel

Modules to use for book outline:
1. **Module 1: The Robotic Nervous System (ROS 2)**
2. **Module 2: The Digital Twin (Gazebo & Unity)**
3. **Module 3: The AI-Robot Brain (NVIDIA Isaac)**
4. **Module 4: Vision-Language-Action (VLA)**

What to generate inside the constitution:
- **Purpose**
- **Vision & Audience**
- **Performance Goals**
- **Non-Goals**
- **Success Criteria**
- **Features & Capabilities**
- **Constraints**
- **Scope (Full 4-Module textbook)**
- **Deliverables (book, RAG chatbot, deployment, optional bonuses)**
- **Risks**
- **Assumptions**
- **Architecture Overview**
- **Document tree for the book**
- **Expected folders for specs, tasks, plans**

Make the constitution complete, detailed, and ready for `/sp.plan` and `/sp.tasks` to run without errors. Ensure folder paths follow Spec-Kit Plus best practices.
```

# Response

```text
I'll help you create a comprehensive constitution for the "Physical AI & Humanoid Robotics – A Complete Textbook" project. Let me start by examining the existing constitution template and then create the updated version based on your requirements.

First, let me check if the constitution file exists:

...

[Response content showing the creation of the comprehensive constitution file with all required sections]
```