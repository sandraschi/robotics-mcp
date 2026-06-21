# Glama.ai Repository Rescan Guide

## How to Trigger Repository Updates on Glama.ai

Since Glama.ai automatically scans repositories, here are the most effective ways to ensure your recent documentation updates (alpha status clarifications) are reflected on their platform.

## 🎯 **Primary Methods to Trigger Rescan**

### 1. **Create a New Release** (Most Effective - Immediate)

```bash
# Create and push a new git tag
git tag -a v0.1.1 -m "docs: clarify alpha status and MCP server dependencies"
git push origin v0.1.1

# Then create a GitHub release
# Go to: https://github.com/sandraschi/robotics-mcp/releases
# Click "Create a new release"
# Tag: v0.1.1
# Title: "Alpha Status Clarification - MCP Server Dependencies"
# Description: 
#   - Updated status from Beta to Alpha
#   - Added prominent warnings about required composited MCP servers
#   - Clarified ongoing development status
#   - Updated all documentation to reflect alpha status
```

**Why this works**: New releases are high-priority triggers for repository scanners and will update the listing within hours.

### 2. **Push Significant Commits** (Fast - Within 24 hours)

```bash
# Make a meaningful commit with the documentation updates
git add .
git commit -m "docs: clarify alpha status and MCP server composition requirements

- Update status from Beta to Alpha across all documentation
- Add prominent warnings about required composited MCP servers
- Clarify that this is ongoing development, not production-ready
- Update pyproject.toml classifier to Development Status :: 3 - Alpha
- Add dedicated MCP Server Dependencies section to README
- Update GLAMA_INTEGRATION.md with alpha status warnings
- Remove misleading 'production ready' language

This addresses concerns about 'plans dressed up as reality' on Glama.ai"
git push origin main
```

**Why this works**: Significant commits trigger daily scans for active repositories.

### 3. **Contact Glama.ai Support** (Manual - Fastest if urgent)

**Email**: support@glama.ai

**Subject**: "Request repository rescan for alpha status clarification - robotics-mcp"

**Message**:
```
Hello Glama.ai Team,

Our repository (sandraschi/robotics-mcp) has recently updated its documentation
to clarify that it is in ALPHA status and requires multiple composited MCP servers
to function. The previous listing may have been misleading about production readiness.

Could you please trigger a rescan to update our listing with the corrected status?

Recent documentation updates include:
- Changed status from Beta to Alpha across all docs
- Added prominent warnings about required composited MCP servers (osc-mcp, unity3d-mcp, vrchat-mcp, avatar-mcp, blender-mcp, gimp-mcp)
- Clarified ongoing development status
- Updated pyproject.toml classifier to Development Status :: 3 - Alpha
- Removed misleading "production ready" language

The README.md now clearly states:
- ⚠️ ALPHA STATUS - ONGOING DEVELOPMENT
- Requires 6+ composited MCP servers
- Features may change or be incomplete

Repository: https://github.com/sandraschi/robotics-mcp
Glama.ai URL: https://glama.ai/mcp/servers/%40sandraschi/robotics-mcp

Thank you!
```

### 4. **Wait for Automatic Scan** (Slowest - Up to 1 week)

Glama.ai automatically scans repositories:
- **Daily**: For repositories with recent activity
- **Weekly**: For all indexed repositories
- **On-demand**: For new releases and major updates

If you've made significant commits, the next daily scan should pick up the changes within 24 hours.

## 📋 **Checklist to Verify Updates**

After triggering a rescan, verify the following on Glama.ai:

- [ ] README shows "ALPHA STATUS - ONGOING DEVELOPMENT" warning
- [ ] Status badge shows "Alpha" (not "Beta" or "Production Ready")
- [ ] MCP Server Dependencies section is visible
- [ ] No mention of "production ready" or "production-ready"
- [ ] pyproject.toml classifier shows "Development Status :: 3 - Alpha"
- [ ] Clear warnings about required composited MCP servers

## 🔍 **How to Check Current Status**

1. Visit: https://glama.ai/mcp/servers/%40sandraschi/robotics-mcp
2. Check the README content displayed
3. Look for status indicators/badges
4. Verify the Overview section mentions alpha status

## ⚡ **Recommended Action Plan**

**For immediate update:**
1. ✅ Push the documentation changes (already done)
2. Create a new release (v0.1.1) with the documentation updates
3. Email Glama.ai support requesting immediate rescan

**For automatic update:**
1. ✅ Push the documentation changes (already done)
2. Wait 24-48 hours for daily scan
3. Verify updates on Glama.ai

## 📝 **What Changed**

The following files were updated to clarify alpha status:

- `README.md`: Added alpha warnings, MCP dependencies section
- `pyproject.toml`: Changed classifier to "Development Status :: 3 - Alpha"
- `docs/GLAMA_INTEGRATION.md`: Updated status references
- `docs/PROGRESS_REPORT.md`: Changed from "Production Ready" to "Alpha"

All changes emphasize:
- ⚠️ ALPHA status (not Beta or Production)
- Requires 6+ composited MCP servers
- Ongoing development - features may change
- Virtual robotics prioritized, physical robots coming later

---

*Last Updated: 2025-12-02*

