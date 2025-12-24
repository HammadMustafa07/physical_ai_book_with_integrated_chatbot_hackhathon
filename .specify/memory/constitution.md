<!--
Sync Impact Report:
Version change: undefined → 1.0.0
Modified principles: None (initial creation)
Added sections: All principles and sections (initial constitution)
Removed sections: None
Templates requiring updates: N/A (first version)
Follow-up TODOs: None
-->
# AI/Spec-Driven Book with Integrated RAG Chatbot Constitution

## Core Principles

### Spec-first development
Every feature and change starts with a specification document; Specifications must be complete, testable, and approved before implementation begins; Clear requirements and acceptance criteria required for all features

### Zero hallucination tolerance
AI responses must be strictly grounded in provided content; No creative extrapolation or assumption of information not present in the source material; Responses must cite specific passages from the book content

### Content-grounded AI responses only
All AI responses must be based solely on indexed book content; If context is missing, respond: 'Information not found in the selected content'; No external knowledge or inference allowed

### Deterministic and auditable behavior
All system behaviors must be predictable and traceable; Responses must be reproducible based on the same input and context; Clear audit trails for all AI interactions

### Production-ready architecture
Architecture must support scalable, reliable production deployment; Proper error handling, monitoring, and operational readiness required; Clean, documented, deployable code standards

### Clear API contracts
Well-defined interfaces between UI, backend, and agent components; Strict contracts for data exchange and service interactions; Clear documentation of all API endpoints and data schemas

## Technology Stack Requirements

Book Platform: Docusaurus
Format: Markdown
Deployment: GitHub Pages
Audience: Developers (AI + full-stack)
RAG Backend: FastAPI
Vector Database: Qdrant Cloud (Free Tier)
Metadata Database: Neon Serverless Postgres
SDK: OpenAI Agents / ChatKit

## Development Workflow

Spec-Kit Plus workflow fully followed
Book builds and deploys successfully
RAG chatbot answers accurately and deterministically
All answers traceable to source passages
Consistent terminology and formatting maintained
Clean, documented, deployable code produced

## Governance

Constitution supersedes all other practices; Amendments require documentation and approval; All implementations must verify compliance with Spec-First development; Complexity must be justified with clear benefits; Use constitution as the guiding document for all architectural decisions

**Version**: 1.0.0 | **Ratified**: 2025-12-20 | **Last Amended**: 2025-12-20