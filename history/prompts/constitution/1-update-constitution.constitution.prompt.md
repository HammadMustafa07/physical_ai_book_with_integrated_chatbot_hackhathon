---
id: 1
title: update-constitution
stage: constitution
date: 2025-12-20
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: main
user: au603
command: sp.constitution
labels: [constitution, spec-first, ai-book]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - null
---

## Prompt

 /sp.constitution

Project: AI/Spec-Driven Book with Integrated RAG Chatbot

Objective:
Create a technical book using Spec-Kit Plus and Claude Code, publish it with Docusaurus on GitHub Pages, and embed a RAG chatbot that answers questions strictly from the book content, including user-selected text.

Core Principles:
- Spec-first development
- Zero hallucination tolerance
- Content-grounded AI responses only
- Deterministic and auditable behavior
- Production-ready architecture

Book Standards:
- Platform: Docusaurus
- Format: Markdown
- Deployment: GitHub Pages
- Audience: Developers (AI + full-stack)
- Style: Clear, instructional, structured

RAG Chatbot Requirements:
- Backend: FastAPI
- Vector DB: Qdrant Cloud (Free Tier)
- Metadata DB: Neon Serverless Postgres
- SDK: OpenAI Agents / ChatKit
- Retrieval limited strictly to indexed book content
- Must support answers based only on user-selected text
- If context is missing, respond: "Information not found in the selected content."

Constraints:
- No external knowledge
- No hard-coded answers
- No creative extrapolation
- Clear API contracts between UI, backend, and agent

Quality Bar:
- All answers traceable to source passages
- Consistent terminology and formatting
- Clean, documented, deployable code

Success Criteria:
- Book builds and deploys successfully
- RAG chatbot answers accurately and deterministically
- Spec-Kit Plus workflow fully followed

## Response snapshot

Created a new constitution file with all the specified principles and requirements for the AI/Spec-Driven Book with Integrated RAG Chatbot project. The constitution includes core principles like spec-first development, zero hallucination tolerance, content-grounded AI responses, deterministic behavior, production-ready architecture, and clear API contracts. It also defines technology stack requirements and development workflow aligned with the project objectives.

## Outcome

- ✅ Impact: Created initial project constitution file at .specify/memory/constitution.md
- 🧪 Tests: No tests required for constitution creation
- 📁 Files: Updated .specify/memory/constitution.md with complete constitution
- 🔁 Next prompts: Plan and implement the book and RAG chatbot features
- 🧠 Reflection: Constitution provides clear governance framework for the project

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable for constitution creation
- Prompt variant (if applicable): Initial constitution creation
- Next experiment (smallest change to try): Begin with feature specification following the constitution principles