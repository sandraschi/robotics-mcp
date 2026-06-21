# Robotics MCP — web dashboard (`web_sota`)

React + TypeScript + Vite. **Lint/format:** [Biome](https://biomejs.dev/) only (`biome.json`). No ESLint.

- **`npm run lint`** — `biome ci .` (CI-style: formatter + linter + import sorting, read-only)
- **`npm run biome`** — `biome check --write .` (local fixes)
- **`npm run biome:ci`** — same as default `lint` alias for automation

See repo root `justfile`: `just lint` runs Ruff on Python **and** Biome in this folder.
