import logging

from fastmcp import Context, FastMCP

logger = logging.getLogger(__name__)


class AgenticSupportTool:
    def __init__(self, mcp: FastMCP):
        self.mcp = mcp

    def register(self):
        @self.mcp.tool()
        async def robotics_agentic_workflow(thought_process: str, objective: str, context: Context) -> str:
            """
            Execute a complex, multi-step robotics workflow leveraging MCP sampling.

            Args:
                thought_process: The reasoning behind invoking this workflow.
                objective: The goal you want to achieve with the robotic fleet.
                context: FastMCP context passed automatically.

            Returns:
                The results of the workflow execution and sampling steps.
            """
            logger.info(f"Agentic workflow initiated: {objective}")

            # Example: Basic FastMCP 3.4.4+ Sampling request
            try:
                msg = [
                    {
                        "role": "user",
                        "content": {
                            "type": "text",
                            "text": f"Evaluate this robotics objective and return a structured JSON mission profile: {objective}",
                        },
                    }
                ]

                # Check for sampling capability
                sampled_response = await context.session.create_message(
                    messages=msg,
                    max_tokens=1000,
                    system_prompt="You are a robotic systems operation intelligence module. Output JSON only.",
                )

                if sampled_response:
                    content = sampled_response.content
                    if isinstance(content, list) and len(content) > 0 and hasattr(content[0], "text"):
                        return f"Mission profile computed via sampling: {content[0].text}"
                    elif hasattr(content, "text"):
                        return f"Mission profile computed via sampling: {content.text}"
                    else:
                        return f"Sampling request returned complex data: {content!s}"

            except Exception as e:
                logger.error(f"Sampling failed: {e!s}")
                return (
                    f"Workflow executed, but sampling was unavailable: {e!s}\n\n"
                    f"I recommend continuing with manual robotic actions."
                )

            return f"Workflow completed for objective: {objective}"
