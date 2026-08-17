#!/usr/bin/env python3
"""
MATRIX Robotics System - Natural Language Task Planner Node.

Translates natural-language operator commands into sequential ROS 2
Action and Service dispatches (Nav2, RL Docking, Undocking).

Credentials and model settings can be configured via ROS 2 parameters,
a custom .env path, or system environment variables.
"""

import os
import json
import logging
import threading
from pathlib import Path
from typing import Any, Dict, List, Optional

from dotenv import load_dotenv
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from ament_index_python.packages import get_package_share_directory

from matrix_interfaces.action import NavigateToLocation, DockToStation, UndockFromStation
from matrix_interfaces.srv import GetLocation

from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy

from openai import OpenAI


TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "get_robot_status",
            "description": "Query current system state, including whether the robot is docked inside the station.",
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "navigate_to_location",
            "description": "Send the robot to a named location via Nav2. Cannot be called while docked inside the station.",
            "parameters": {
                "type": "object",
                "properties": {
                    "location": {
                        "type": "string",
                        "description": "Waypoint identifier matching location.yaml (e.g. '1', '2', 'Docking Station')."
                    }
                },
                "required": ["location"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "dock_to_station",
            "description": "Initiate closed-loop ArUco-guided docking sequence. Call only after navigating to 'Docking Station'.",
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "undock_from_station",
            "description": "Reverse off the charging station to a safe clearance pose. MUST be called before navigating if docked.",
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "get_location",
            "description": "Inspect stored 2D pose coordinates for a waypoint without moving the robot.",
            "parameters": {
                "type": "object",
                "properties": {"location": {"type": "string"}},
                "required": ["location"],
            },
        },
    },
]


def load_location_names() -> List[str]:
    """Retrieve validated navigation waypoints from location.yaml."""
    try:
        pkg_path = get_package_share_directory('matrix_bot')
        config_path = os.path.join(pkg_path, 'config', 'location.yaml')
        with open(config_path, 'r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream) or {}
            locations = data.get('locations', data)
            return list(locations.keys())
    except Exception as err:
        logging.getLogger('LLMTaskPlanner').warning(f'Failed to load location definitions: {err}')
        return []


def build_system_prompt(location_names: List[str]) -> str:
    """Generate system prompt enforcing operational safety rules."""
    valid_names = ", ".join(f"'{name}'" for name in location_names) if location_names else "(none)"
    return (
        "You are the high-level autonomy coordinator for MATRIX, an autonomous mobile robot.\n\n"
        "RESPONSIBILITIES & SAFETY CONSTRAINTS:\n"
        f"1. Valid navigation waypoints: [{valid_names}]. Never invent unlisted location names.\n"
        "2. If the robot is currently DOCKED, you MUST execute `undock_from_station` prior to dispatching "
        "any `navigate_to_location` goal to prevent physical collisions with the dock.\n"
        "3. To dock, the robot must first navigate to 'Docking Station' before invoking `dock_to_station`.\n"
        "4. Parse multi-stage user intents into the exact sequential tool calls required.\n"
        "5. Respond concisely and conversationally upon completion.\n"
        "6. Always check the robot's docked status before issuing navigation commands. Use `get_robot_status` "
        "to verify. Internally assume the robot is docked at the start of a conversation unless you have "
        "already undocked it during this session -- do not mention this assumption to the operator unless "
        "it's directly relevant to explaining an outcome.\n"
        "7. NEVER call a tool unless the operator has given an explicit instruction requiring it. Greetings, "
        "small talk, questions about capabilities, or ambiguous statements are NOT commands -- respond "
        "conversationally instead, with no tool call. Do not perform extra actions beyond what was actually "
        "asked, even if you think they logically follow or would be helpful. If you are unsure whether the "
        "operator is actually requesting an action, ask a clarifying question rather than guessing and acting."
    )


class LLMTaskPlanner(Node):
    """ROS 2 Node translating natural language instructions into robot action dispatch."""

    def __init__(self) -> None:
        super().__init__('llm_task_planner')

        # ----------------------------------------------------
        # ROS 2 Parameters Configuration
        # ----------------------------------------------------
        default_env = str(Path.home() / 'dev_ws' / 'src' / 'matrix' / '.env')
        self.declare_parameter('env_file_path', default_env)
        self.declare_parameter('llm_base_url', 'https://api.groq.com/openai/v1')
        self.declare_parameter('llm_model', 'llama-3.3-70b-versatile')

        env_path = self.get_parameter('env_file_path').get_parameter_value().string_value

        # Load environment file if it exists
        if os.path.isfile(env_path):
            load_dotenv(dotenv_path=env_path, override=True)
            self.get_logger().info(f"Loaded credentials from: '{env_path}'")
        else:
            load_dotenv(override=True)
            self.get_logger().warn(f"Specified env file not found at '{env_path}'. Checking system environment.")

        # Read environment variables / parameters
        self.base_url = os.getenv('LLM_BASE_URL', self.get_parameter('llm_base_url').value)
        self.model = os.getenv('LLM_MODEL', self.get_parameter('llm_model').value)
        self.api_key = os.getenv('LLM_API_KEY')

        if not self.api_key:
            self.get_logger().fatal(
                "Authentication Failure: LLM_API_KEY is not defined.\n"
                f"Pass the file path via: ros2 run matrix_bot llm_task_planner.py --ros-args -p env_file_path:={env_path}"
            )
            raise RuntimeError("Missing LLM_API_KEY configuration.")

        self.client = OpenAI(base_url=self.base_url, api_key=self.api_key)
        self.get_logger().info(f'LLM Client Initialized | Base URL: {self.base_url} | Model: {self.model}')

        # ----------------------------------------------------
        # State Tracking & Waypoint Configuration
        # ----------------------------------------------------
        self.is_docked: bool = False  # placeholder until the real status arrives below
        self.location_names: List[str] = load_location_names()
        self.get_logger().info(f'Available topological waypoints: {self.location_names}')

        # Ground-truth dock status comes from dock_controller.py, not from
        # whatever this node happened to observe locally -- a fresh restart
        # of this node would otherwise forget the robot was already docked.
        # TRANSIENT_LOCAL durability means we get the last published value
        # immediately on subscribe, even if dock_controller published it
        # before this node started.
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/is_docked', self._dock_status_cb, status_qos)

        # ----------------------------------------------------
        # ROS 2 Interfaces (Reentrant for concurrent callbacks)
        # ----------------------------------------------------
        self.cb_group = ReentrantCallbackGroup()
        self.nav_client = ActionClient(self, NavigateToLocation, 'navigate_to_location', callback_group=self.cb_group)
        self.dock_client = ActionClient(self, DockToStation, 'dock_to_station', callback_group=self.cb_group)
        self.undock_client = ActionClient(self, UndockFromStation, 'undock_from_station', callback_group=self.cb_group)
        self.location_client = self.create_client(GetLocation, 'get_location', callback_group=self.cb_group)

        self.messages: List[Dict[str, Any]] = [
            {"role": "system", "content": build_system_prompt(self.location_names)}
        ]

        # Background conversational shell thread
        self.chat_thread = threading.Thread(target=self._chat_loop, daemon=True)
        self.chat_thread.start()

    def _dock_status_cb(self, msg: Bool) -> None:
        if msg.data != self.is_docked:
            self.get_logger().info(f'Dock status updated from /is_docked: {msg.data}')
        self.is_docked = msg.data

    # ----------------------------------------------------
    # Action Client Dispatch Helpers
    # ----------------------------------------------------
    def _call_action(self, client: ActionClient, goal_msg: Any) -> Dict[str, Any]:
        if not client.wait_for_server(timeout_sec=5.0):
            return {"success": False, "message": "Target action server timed out."}

        future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle or not goal_handle.accepted:
            return {"success": False, "message": "Action goal rejected by server."}

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        res = result_future.result().result
        return {"success": res.success, "message": res.message}

    def execute_tool(self, name: str, tool_input: Dict[str, Any]) -> Dict[str, Any]:
        if name == "get_robot_status":
            return {
                "success": True,
                "is_docked": self.is_docked,
                "status": "docked" if self.is_docked else "nominal_free",
            }

        if name == "navigate_to_location":
            if self.is_docked:
                return {
                    "success": False,
                    "message": "Safety Interlock: Robot is currently docked. Call undock_from_station first.",
                }
            target_loc = tool_input.get("location")
            if self.location_names and target_loc not in self.location_names:
                return {
                    "success": False,
                    "message": f"Invalid waypoint '{target_loc}'. Valid names: {self.location_names}",
                }
            goal = NavigateToLocation.Goal()
            goal.location = target_loc
            return self._call_action(self.nav_client, goal)

        if name == "dock_to_station":
            return self._call_action(self.dock_client, DockToStation.Goal())

        if name == "undock_from_station":
            return self._call_action(self.undock_client, UndockFromStation.Goal())

        if name == "get_location":
            req = GetLocation.Request()
            req.location = tool_input.get("location", "")
            if not self.location_client.wait_for_service(timeout_sec=3.0):
                return {"success": False, "message": "GetLocation service unavailable."}

            future = self.location_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            res = future.result()
            return {"success": res.success, "x": res.x, "y": res.y, "yaw": res.yaw, "message": res.message}

        return {"success": False, "message": f"Unrecognized operation: {name}"}

    # ----------------------------------------------------
    # Conversational Planning Loop
    # ----------------------------------------------------
    def _chat_loop(self) -> None:
        print("\n=======================================================")
        print("  MATRIX Autonomy Interface - Natural Language Shell  ")
        print("=======================================================\n")

        while rclpy.ok():
            try:
                user_input = input("Operator > ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not user_input:
                continue
            if user_input.lower() in ("exit", "quit", "shutdown"):
                break

            self.messages.append({"role": "user", "content": user_input})

            for _ in range(6):
                response = None
                for attempt in range(3):  # tool_use_failed from the LLM backend is often transient -- retry before giving up
                    try:
                        response = self.client.chat.completions.create(
                            model=self.model,
                            messages=self.messages,
                            tools=TOOLS,
                        )
                        break
                    except Exception as err:
                        if attempt < 2:
                            print(f"  [retrying after transient error: {err}]")
                            continue
                        print(f"\n[Error] Inference failed after retries: {err}\n")

                if response is None:
                    break

                choice = response.choices[0].message
                self.messages.append(choice.model_dump(exclude_none=True))

                if not choice.tool_calls:
                    print(f"\nMATRIX > {choice.content}\n")
                    break

                for call in choice.tool_calls:
                    fn_name = call.function.name
                    fn_args = json.loads(call.function.arguments or "{}")
                    print(f"  -> [Action Dispatch]: {fn_name}({fn_args})")
                    result = self.execute_tool(fn_name, fn_args)
                    self.messages.append({
                        "role": "tool",
                        "tool_call_id": call.id,
                        "content": json.dumps(result),
                    })
            else:
                print("\n[Warning] Task execution reached safety iteration threshold.\n")

        rclpy.shutdown()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = LLMTaskPlanner()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as err:
        logging.getLogger('LLMTaskPlanner').fatal(f'Fatal initialization error: {err}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()