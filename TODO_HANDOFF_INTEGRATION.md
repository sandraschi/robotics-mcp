# TODO: Agent-to-Agent Handoff Integration (SOTA 2025)

This document outlines the requirements for implementing behavioral delegation between autonomous agents in the `robotics-mcp` server, following the Dec 2025 SOTA patterns.

## 🏁 Goal
Enable the Robotics Agent to accept handoffs from a Generalist Assistant (e.g., `vienna-lfe-assistant`) and delegate to specialized sub-agents (e.g., `philosophical-engine`, `visualization-specialist`).

---

## 🛠️ MCP Tool Implementation
- [ ] **Create `handoff_to_agent` Tool**:
    - Parameters: `target_agent_id`, `semantic_context`, `identity_token`.
    - Returns: `HandoffResult` with status `HANDOFF_PENDING` or `HANDOFF_COMPLETE`.
- [ ] **Implement `accept_handoff` Handler**:
    - Logic to deserialize incoming context into the local `RobotStateManager`.
    - Validation of `identity_token` (JWT/OIDC).

## 📄 Context Serialization
- [ ] **Context Service**:
    - Implement a service to extract "Semantic Snapshots" of the current robot state (e.g., active robot ID, last motion command, active errors).
    - Ensure context does not exceed 4k tokens to fit within target agent prompts.

## 🔐 Identity Propagation
- [ ] **Token Validation Middleware**:
    - Integrate with the existing `config/auth.yaml` (if present) to verify signed claims.
    - Implement "Identity Proxying": The Robotics Agent should be able to call other servers using the user's forwarded identity.

## 🧪 Integration Tests
- [ ] Create a mock "Generalist Agent" to test the `POST /handoff` / `accept_handoff` flow.
- [ ] Verify that `RobotStateManager` correctly populates from transferred semantic context.

---

## 📚 Reference
See [AGENT_TO_AGENT_HANDOFF.md](file:///d:/Dev/repos/mcp-central-docs/docs/patterns/AGENT_TO_AGENT_HANDOFF.md) in the `mcp-central-docs` repository for the full pattern specification.
