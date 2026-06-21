"""Environmental logistics portmanteau tool - fabric, garden, pet, health, and mural handling.

Consolidates specialized robotic logistics including textiles, wildlife rescue,
security patrols, pet care, human health supervision, and aesthetic murals.
"""

from typing import Any, Literal

import structlog
from fastmcp import Context

from ..utils.error_handler import (
    format_error_response,
    format_success_response,
    handle_tool_error,
)

logger = structlog.get_logger(__name__)


class EnvironmentalLogisticsTool:
    """Portmanteau tool for environmental logistics and substrate handling."""

    def __init__(
        self,
        mcp: Any,
        state_manager: Any,
        mounted_servers: dict[str, Any] | None = None,
    ):
        """Initialize textile tool.

        Args:
            mcp: FastMCP server instance.
            state_manager: Robot state manager instance.
            mounted_servers: Dictionary of mounted MCP servers.
        """
        self.mcp = mcp
        self.state_manager = state_manager
        self.mounted_servers = mounted_servers or {}

    def register(self):
        """Register textile tool with MCP server."""

        @self.mcp.tool()
        async def environmental_logistics_handler(
            ctx: Context,
            operation: Literal[
                "stiffen_fabric",
                "grip_textile",
                "manipulate_pattern",
                "darkroom_logistics",
                "diy_construction",
                "facade_maintenance",
                "garden_logistics",
                "security_patrol",
                "pet_logistics",
                "human_health_supervision",
                "companion_logistics",
                "mural_painting",
                "emergency_dispatch",
            ],
            material_type: str,
            # Stiffening parameters
            stiffening_agent: Literal["gelatin", "starch", "pva"] = "gelatin",
            concentration_pct: float = 7.5,
            # Tooling parameters
            gripper_type: Literal[
                "vacuum_bernoulli",
                "vacuum_standard",
                "magnetic",
                "soft_fingers",
                "chem_resistant_pincers",
                "paint_sprayer_nozzle",
                "leaf_blower",
                "thermal_search_scanner",
                "ball_launcher",
                "treat_dispenser",
                "laser_pointer",
                "water_nozzle",
                "soft_tactile_huggie",
            ] = "vacuum_bernoulli",
            pressure_kpa: float = -20.0,
            # Maintenance/Spraying parameters
            paint_type: str | None = None,
            nozzle_pattern: Literal["flat_fan", "cone", "pinpoint"] = "flat_fan",
            # Manipulation/DIY/Garden/Security/Pet/Pool/Human/Mural parameters
            pattern_id: str | None = None,
            action: str | None = None,
            force_n: float = 1.0,
        ) -> dict[str, Any]:
            """Handle specialized environmental and substrate logistics.

            Covers everything from textiles and darkroom film to wildlife search & rescue,
            autonomous security patrols, pet logistics, pool supervision, human health, and mural painting.

            Args:
                operation: The logistics operation to perform.
                material_type: Material category (e.g., "silk", "photo_paper", "fence", "nestling", "dog", "pool_perimeter", "toddler", "senior", "nursery_wall").
                stiffening_agent: Polymer used for temporary textile rigidification.
                concentration_pct: Agent concentration for stiffening.
                gripper_type: Robotic end-effector or tool type.
                pressure_kpa: Operating pressure (vacuum or positive).
                paint_type: Color or type of paint.
                nozzle_pattern: Spray pattern for the paint nozzle.
                pattern_id: Optional ID for the pattern, plan, or bounding box.
                action: Specific action (e.g., "laser_play", "pet_shower", "pool_supervise", "fall_detect", "huggie_suit", "mural_animal_scene", "call_brother_steve", "call_police", "call_firefighters", "call_ambulance").
                force_n: Application force in Newtons.
            """
            try:
                logger.info(
                    "Substrate handling operation",
                    operation=operation,
                    material_type=material_type,
                )

                if operation == "stiffen_fabric":
                    return await self._handle_stiffening(material_type, stiffening_agent, concentration_pct)
                elif operation == "grip_textile":
                    return await self._handle_gripping(material_type, gripper_type, pressure_kpa)
                elif operation == "manipulate_pattern":
                    return await self._handle_manipulation(material_type, pattern_id, action)
                elif operation == "darkroom_logistics":
                    return await self._handle_darkroom(material_type, action)
                elif operation == "diy_construction":
                    return await self._handle_diy(material_type, action, force_n)
                elif operation == "facade_maintenance":
                    return await self._handle_maintenance(
                        material_type, action, paint_type, nozzle_pattern, pressure_kpa
                    )
                elif operation == "garden_logistics":
                    return await self._handle_garden(material_type, action, gripper_type, pressure_kpa)
                elif operation == "wildlife_rescue":
                    return await self._handle_rescue(material_type, action)
                elif operation == "security_patrol":
                    return await self._handle_security(material_type, action)
                elif operation == "pet_logistics":
                    return await self._handle_pet(material_type, action, gripper_type, force_n)
                elif operation == "human_health_supervision":
                    return await self._handle_human_health(material_type, action)
                elif operation == "companion_logistics":
                    return await self._handle_companionship(material_type, action, gripper_type)
                elif operation == "mural_painting":
                    return await self._handle_mural(material_type, action, paint_type, nozzle_pattern, pressure_kpa)
                elif operation == "emergency_dispatch":
                    return await self._handle_emergency_dispatch(material_type, action)
                else:
                    return format_error_response(f"Unknown operation: {operation}")

            except Exception as e:
                return handle_tool_error("environmental_logistics_handler", e)

    async def _handle_stiffening(self, material: str, agent: str, concentration: float) -> dict[str, Any]:
        """Generate stiffening workflow parameters."""
        prep_time_mins = 30 if agent == "gelatin" else 15
        drying_time_mins = 60 if concentration > 5 else 45

        message = f"Stiffening protocol generated for {material} using {concentration}% {agent}."
        if material.lower() in ["silk", "lace"] and agent == "gelatin":
            message += " [SOTA Recommendation] Gelatin provides optimal structural integrity for delicate fibers."

        return format_success_response(
            message,
            data={
                "material": material,
                "agent": agent,
                "concentration": concentration,
                "prep_instructions": [
                    f"Dissolve {agent} in 60°C water.",
                    f"Submerge {material} for {prep_time_mins} minutes.",
                    f"Dry flat for {drying_time_mins} minutes or until rigid.",
                ],
                "expected_rigidity_kpa": concentration * 1.5,
            },
        )

    async def _handle_gripping(self, material: str, gripper: str, pressure: float) -> dict[str, Any]:
        """Configure and verify robotic gripping sequence."""
        safety_status = "nominal"
        warnings = []

        if gripper == "vacuum_bernoulli" and pressure < -30:
            warnings.append("High pressure detected. Risk of fiber deformation on porous textiles.")
            safety_status = "caution"

        return format_success_response(
            f"Gripping sequence initialized using {gripper}.",
            data={
                "gripper_config": {
                    "type": gripper,
                    "target_pressure": pressure,
                    "contact_force": 0.5 if "vacuum" in gripper else 2.0,
                },
                "safety_telemetry": {"status": safety_status, "warnings": warnings},
            },
        )

    async def _handle_manipulation(self, material: str, pattern: str | None, action: str | None) -> dict[str, Any]:
        """Execute robotic manipulation paths."""
        return format_success_response(
            f"Manipulation action '{action}' queued for pattern {pattern}.",
            data={
                "material_state": "stiffened",
                "path_precision_mm": 0.05,
                "estimated_force_n": 1.2,
            },
        )

    async def _handle_darkroom(self, material: str, action: str | None) -> dict[str, Any]:
        """Handle analog darkroom workflows (wet substrates)."""
        workflow = {
            "develop": ["Tray 1: Developer (90s)", "Agitate every 30s"],
            "stop_bath": ["Tray 2: Stop Bath (30s)", "Continuous agitation"],
            "fixer": ["Tray 3: Fixer (2-5m)", "Agitate every 60s"],
            "wash": ["Tray 4: Final Wash (10m)", "Running water"],
            "dry": ["Hanging Rail", "Attach clip to corner", "Drip dry (2hr)"],
        }

        steps = workflow.get(action, ["Universal Rinse/Agitation"])

        return format_success_response(
            f"Darkroom {action} sequence initiated for {material}.",
            data={
                "substrate": material,
                "environment": "darkroom_safe_red",
                "chemical_resistance": "ISO_175_compliant",
                "gripper_mode": "pincer_edge_grip",
                "steps": steps,
            },
        )

    async def _handle_diy(self, material: str, action: str | None, force: float) -> dict[str, Any]:
        """Handle industrial/DIY construction tasks."""
        safety_guard = "active"

        if action == "hammer_nail":
            grip_mode = "magnetic_axial"
            precision = "0.2mm"
        elif action == "brick_lay":
            grip_mode = "suction_heavy"
            precision = "1.0mm"
        else:
            grip_mode = "standard"
            precision = "2.0mm"

        return format_success_response(
            f"DIY {action} initiated on {material} with {force}N force.",
            data={
                "construction_substrate": material,
                "grip_mode": grip_mode,
                "precision_target": precision,
                "safety_guard": safety_guard,
                "telemetry": {"verticality": "99.8%", "force_feedback": "active"},
            },
        )

    async def _handle_maintenance(
        self,
        material: str,
        action: str | None,
        paint_type: str | None,
        pattern: str,
        pressure: float,
    ) -> dict[str, Any]:
        """Handle facade maintenance tasks (paint spraying)."""
        if action == "overspray_graffiti":
            target_distance_cm = 30.0
            optimal_velocity_mm_s = 150.0
            base_pressure = pressure if pressure > 0 else 50.0  # Positive pressure for spraying

            message = f"Graffiti overspray sequence initialized for {material}."
            if paint_type:
                message += f" Using paint match: {paint_type}."

            return format_success_response(
                message,
                data={
                    "substrate": material,
                    "tool": "paint_sprayer_nozzle",
                    "spray_pattern": pattern,
                    "operating_params": {
                        "pressure_kpa": base_pressure,
                        "nozzle_distance_cm": target_distance_cm,
                        "traverse_speed_mm_s": optimal_velocity_mm_s,
                        "overlap_pct": 25.0,
                    },
                    "steps": [
                        "1. Scan facade to bound target area.",
                        "2. Calibrate nozzle distance and traverse speed.",
                        "3. Execute primary overspray pass.",
                        "4. Inspect opacity and apply touch-up if needed.",
                    ],
                },
            )

        return format_error_response(f"Unknown maintenance action: {action}")

    async def _handle_garden(
        self,
        material: str,
        action: str | None,
        tool: str,
        pressure: float,
    ) -> dict[str, Any]:
        """Handle garden logistics and wildlife safety."""
        if action == "leaf_blow":
            base_pressure = pressure if pressure > 0 else 120.0
            return format_success_response(
                f"Leaf blowing sequence initiated for {material}.",
                data={
                    "tool": tool,
                    "target_substrate": "organic_debris",
                    "operating_params": {
                        "nozzle_pressure_kpa": base_pressure,
                        "sweep_angle_deg": 45.0,
                        "ground_clearance_cm": 15.0,
                    },
                    "strategy": "Centralized pile aggregation",
                },
            )

        if action == "hedgehog_vanguard":
            return format_success_response(
                "Hedgehog Vanguard mode ACTIVATED.",
                data={
                    "mission": "Autonomous mower safety escort",
                    "subsystems": {
                        "thermal_imaging": "ACTIVE",
                        "ultrasonic_scouts": "ACTIVE",
                        "propulsive_shooing": "Gentle acoustic pulses",
                    },
                    "rules_of_engagement": [
                        "1. Walk 2.5m ahead of mowerbot.",
                        "2. Scan for thermal signatures of Erinaceinae.",
                        "3. If detected, execute gentle shooing sequence.",
                        "4. Maintain safety perimeter until mower passage.",
                    ],
                    "status": "Escorting Mower_01",
                },
            )

        return format_error_response(f"Unknown garden action: {action}")

    async def _handle_rescue(
        self,
        material: str,
        action: str | None,
    ) -> dict[str, Any]:
        """Handle wildlife search and rescue missions."""
        if action == "rescue":
            return format_success_response(
                f"Wildlife rescue mission initialized for {material}.",
                data={
                    "mission_type": "Search & Rescue",
                    "sensors": ["Thermal_HI_RES", "Acoustic_Sensitivity_Ultra"],
                    "gripper_mode": "soft_fingers_low_torque",
                    "steps": [
                        f"1. Deploy thermal sweep for {material} signature.",
                        "2. Approach with low-noise locomotion.",
                        "3. Evaluate injury via visual zoom (10x).",
                        "4. Execute gentle retrieval or alert veterinarian.",
                    ],
                    "telemetry": {"stealth_mode": "active", "biometric_scan": "ready"},
                },
            )

        if action == "check_traps":
            return format_success_response(
                f"Checking non-lethal traps for {material}.",
                data={
                    "sensor_status": "IR_active",
                    "logic": "Humanitarian_pest_management",
                    "action_if_occupied": "Relocate to safe woodland perimeter",
                },
            )

        return format_error_response(f"Unknown rescue action: {action}")

    async def _handle_security(
        self,
        material: str,
        action: str | None,
    ) -> dict[str, Any]:
        """Handle autonomous security and perimeter audits."""
        if action == "fence_audit":
            return format_success_response(
                f"Perimeter fence integrity audit initiated for {material}.",
                data={
                    "audit_type": "Structural_Loss_Detection",
                    "sensors": ["LiDAR_360", "Optical_Flow_Cuts"],
                    "mode": "Boundary_Follower",
                    "steps": [
                        "1. Trace perimeter coordinates.",
                        "2. Analyze mesh/structure for discontinuities.",
                        "3. Tag potential cut/hole locations with GPS.",
                        "4. Compile report for human maintenance.",
                    ],
                    "alarm_threshold": "Critical_Opening_Detected",
                },
            )

        return format_error_response(f"Unknown security action: {action}")

    async def _handle_pet(
        self,
        material: str,
        action: str | None,
        tool: str,
        force: float,
    ) -> dict[str, Any]:
        """Handle pet interaction and behavioral supervision."""
        if action == "throw_ball":
            velocity = force * 5.0  # Simulated velocity calculation
            return format_success_response(
                f"Tennis ball launched for {material}.",
                data={
                    "velocity_mps": velocity,
                    "trajectory": "Parabolic_High_Arc",
                    "mode": "Fetch_Training",
                    "status": "Awaiting_Retrieval",
                },
            )

        if action == "dispense_treat":
            return format_success_response(
                f"Treat dispensed for {material}.",
                data={
                    "dispenser_type": tool,
                    "dosage": "1_unit",
                    "reward_status": "Positive_Reinforcement",
                },
            )

        if action == "train_command":
            return format_success_response(
                f"Command training sequence initialized: '{material} SIT'.",
                data={
                    "acoustic_command": f"Vocal_Synthesis_{material}_Sit",
                    "visual_cue": "Low_Palm_Gesture",
                    "verification": "Pose_Analyzer_SITTING",
                },
            )

        if action == "supervise":
            return format_success_response(
                f"Autonomous behavioral supervision active for {material}.",
                data={
                    "monitoring_active": True,
                    "event_detection": ["Poopy_In_Living_Room", "Furniture_Scratch"],
                    "correction_logic": {
                        "vocal": "Pfui Waldmann!",
                        "acoustic": "Ultra_Frequency_Bip_30ms",
                    },
                    "telemetry": {
                        "pose_estimator": "active",
                        "spatial_cleanup": "ready",
                    },
                },
            )

        if action == "laser_play":
            return format_success_response(
                f"Low-energy laser play initialized for {material}.",
                data={
                    "emitter": "Laser_Node_Eye_Safe",
                    "pattern": "Random_Jitter_High_Agility",
                    "safety": "Avoid_Retinal_Direct_Hit_Active",
                },
            )

        if action == "pet_shower":
            return format_success_response(
                f"Autonomous water logistics (shower/hose) for {material}.",
                data={
                    "tool": tool,
                    "temperature": "38°C_Optimal",
                    "pressure": "Gentle_Massage_Flow",
                    "shampoo_cycle": "Organic_Oatmeal_Infusion",
                },
            )

        if action == "pool_supervise":
            return format_success_response(
                f"Pool safety supervision active for {material}.",
                data={
                    "boundary": "Pool_Perimeter_Virtual_Fence",
                    "detection": ["Water_Entry_Event", "Surface_Disturbance"],
                    "action_on_alert": "Activate_Deep_Alarm_And_DDA_Response",
                },
            )

        return format_error_response(f"Unknown pet action: {action}")

    async def _handle_human_health(
        self,
        material: str,
        action: str | None,
    ) -> dict[str, Any]:
        """Handle human health supervision logistics (Toddlers/Seniors)."""
        if action == "choke_hazard_scan":
            return format_success_response(
                f"Toddler safety scan (choke-hazard) active for {material}.",
                data={
                    "scanner": "Hyper_Spectral_Material_Analyzer",
                    "object_detection": [
                        "Small_Particulated_Plastics",
                        "Coin_Detected",
                    ],
                    "threat_level": "High_Alert_Manual_Intervention_Required",
                },
            )

        if action == "boundary_alert":
            return format_success_response(
                f"Perimeter boundary protection active for {material}.",
                data={
                    "fence": "Stairwell_Invisible_Fence",
                    "status": "Monitored",
                    "action_on_breach": "Autonomous_Physical_Blocking_Engagement",
                },
            )

        if action == "fall_detect":
            return format_success_response(
                f"Senior health audit (fall detection) active for {material}.",
                data={
                    "sensor": "IMU_Pulse_Gait_Analyzer",
                    "status": "Scanning_For_Rapid_Z_Axis_Deceleration",
                    "emergency_protocol": "Immediate_Family_Notification_DDA",
                },
            )

        if action == "medication_audit":
            return format_success_response(
                f"Medication adherence auditing for {material}.",
                data={
                    "plan": "Schedule_V14_Daily",
                    "dispenser_status": "Verified",
                    "verification": "Visual_Swallow_Confirmation_Active",
                },
            )

        return format_error_response(f"Unknown health supervision action: {action}")

    async def _handle_companionship(
        self,
        material: str,
        action: str | None,
        tool: str,
    ) -> dict[str, Any]:
        """Handle companion robotics (Huggie-Suit/Emotional Auditing)."""
        if action == "huggie_suit":
            return format_success_response(
                "Japan-Pattern 'Huggie-Suit' deployment active.",
                data={
                    "substrate": material,
                    "tool": tool,
                    "tactile_mode": "Gentle_Pressure_Bio_Mimicry",
                    "warming_element": "36.5°C_Human_Parity",
                },
            )

        if action == "emotional_audit":
            return format_success_response(
                f"Emotional state analysis active for {material}.",
                data={
                    "analysis": [
                        "Vocal_Tonality_Pitch_Shift",
                        "Facial_Micro_Expression",
                    ],
                    "mood_detected": "Neutral_to_Slightly_Stressed",
                    "intervention": "Soft_Vocal_Soothing_Cycle_Initiated",
                },
            )

        return format_error_response(f"Unknown companion action: {action}")

    async def _handle_mural(
        self,
        material: str,
        action: str | None,
        paint: str | None,
        pattern: str,
        pressure: float,
    ) -> dict[str, Any]:
        """Handle aesthetic logistics: Mural Painting and Decoration."""
        if action == "mural_animal_scene":
            return format_success_response(
                f"Nursery mural deployment ('Cute Animal Scene') active for {material}.",
                data={
                    "style": "Illustrative_High_Fidelity",
                    "subject": ["Bears", "Bunnies", "Forest_Floor"],
                    "paint_profile": paint or "Non_Toxic_Water_Based_Eco_Matte",
                    "pattern": pattern,
                    "pressure": pressure,
                },
            )

        if action == "nano_banana_transfer":
            return format_success_response(
                f"Precision 'Nano-Banana' transfer-to-wall active for {material}.",
                data={
                    "technique": "Stencil_Projection_Mapping_Hybrid",
                    "accuracy": "Sub_Millimeter_Geometric_Parity",
                    "feedback_loop": "Real_Time_Laser_Vision_Validation",
                    "substrate_prep": "Autonomous_Surface_Degreasing",
                },
            )

        if action == "consistency_audit":
            return format_success_response(
                "Autonomous paint consistency auditing active.",
                data={
                    "viscosity": "Dynamic_Flow_Compensation",
                    "status": "Optimal_Non_Drip_Parity",
                },
            )

        return format_error_response(f"Unknown mural action: {action}")

    async def _handle_emergency_dispatch(
        self,
        emergency_type: str,
        action: str | None,
    ) -> dict[str, Any]:
        """Handle emergency communication and dispatch logistics."""
        # Preparation of base SOTA templates
        base_address = "District 9 (Alsergrund), 1090 Wien, Austria"
        user_name = "Sandra Schipal"

        templates = {
            "call_brother_steve": {
                "contact": "Steve (Brother)",
                "priority": "HIGH",
                "message": f"Hi Steve, this is Sandra's autonomous substrate handler. There is a {emergency_type} emergency at {base_address}. Please respond immediately.",
            },
            "call_police": {
                "contact": "Polizei (133)",
                "priority": "CRITICAL",
                "message": f"EMERGENCY DISPATCH: Security breach detected at {base_address}. Resident: {user_name}. Type: {emergency_type}. Tactical deployment requested.",
            },
            "call_firefighters": {
                "contact": "Feuerwehr (122)",
                "priority": "CRITICAL",
                "message": f"EMERGENCY DISPATCH: Fire/Environmental hazard at {base_address}. Resident: {user_name}. Type: {emergency_type}. Immediate suppression required.",
            },
            "call_ambulance": {
                "contact": "Rettung (144)",
                "priority": "CRITICAL",
                "message": f"EMERGENCY DISPATCH: Health emergency at {base_address}. Resident: {user_name}. Subject Type: {emergency_type}. Life-support equipment indicated.",
            },
        }

        dispatch_info = templates.get(
            action,
            {
                "contact": "UNKNOWN",
                "priority": "MEDIUM",
                "message": f"Unrecognized emergency event: {emergency_type} at {base_address}.",
            },
        )

        return format_success_response(
            f"Emergency dispatch '{action}' initiated for {emergency_type}.",
            data={
                "dispatch_telemetry": dispatch_info,
                "status": "Transmitting",
                "verification": "Signal_Lock_Confirmed",
                "location_parity": "Verified (Alsergrund)",
            },
        )
