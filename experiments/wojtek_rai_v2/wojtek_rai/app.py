"""Streamlit chat with Wojtek, streaming straight from LangGraph.

    ./experiments/wojtek_rai_v2/run.sh agent      ->  http://localhost:8501

Every LLM token, every tool call (with its arguments) and every tool result
is rendered the moment LangGraph emits it, so a slow step is visibly slow
instead of a blank spinner. Turn plumbing lives in wojtek_rai.stream.
"""

from __future__ import annotations

import json
import time
from typing import Any, Dict, List

import rclpy
import streamlit as st
from langchain_core.messages import AIMessage, BaseMessage, HumanMessage, ToolMessage
from rai.communication.ros2 import ROS2Connector

from wojtek_rai.agent import READ_ONLY, build_agent
from wojtek_rai.camera_feed import start_camera_feed
from wojtek_rai.stream import TurnEvents, run_turn, text_of

CAMERA_REFRESH_S = 0.5

GREETING = (
    "Hi, I am Wojtek. I can stand up, walk, turn, look through my camera and "
    "report where I am. What should I do?"
    if not READ_ONLY
    else "Hi, I am Wojtek in read-only mode: I can look through my camera and "
    "report, but I have no movement tools in this session."
)

st.set_page_config(page_title="Wojtek x RAI", page_icon=":dog:", layout="wide")


@st.cache_resource
def get_graph():
    if not rclpy.ok():
        rclpy.init()
    connector = ROS2Connector(node_name="wojtek_rai_agent", executor_type="multi_threaded")
    return build_agent(connector).agent


@st.cache_resource
def get_camera_feed():
    if not rclpy.ok():
        rclpy.init()
    feed = start_camera_feed()
    time.sleep(0.6)  # let the first frames arrive before the first render
    return feed


@st.fragment(run_every=CAMERA_REFRESH_S)
def camera_panel() -> None:
    """Live colour + depth preview, re-rendered on its own timer."""
    color, color_age, depth, depth_age = get_camera_feed().latest()
    show_depth = st.session_state.get("show_depth", False)
    if color is None:
        st.info("no camera frames yet")
    else:
        st.image(color, caption=f"colour, {color_age:.1f} s old", use_container_width=True)
    if show_depth:
        if depth is None:
            st.info("no depth frames yet")
        else:
            st.image(depth, caption=f"depth (bright = near), {depth_age:.1f} s old", use_container_width=True)


def _render_tool_result(msg: ToolMessage) -> None:
    with st.expander(f"tool result: {msg.name}", expanded=False):
        st.code(text_of(msg.content)[:4000])
        artifact = getattr(msg, "artifact", None)
        images = artifact.get("images") if isinstance(artifact, dict) else None
        for img in images or []:
            st.image(f"data:image/png;base64,{img}" if isinstance(img, str) else img)


def _render_history(messages: List[BaseMessage]) -> None:
    for msg in messages:
        if isinstance(msg, HumanMessage):
            st.chat_message("user").write(text_of(msg.content))
        elif isinstance(msg, AIMessage):
            with st.chat_message("assistant"):
                for call in msg.tool_calls or []:
                    st.code(f"{call['name']}({json.dumps(call['args'])})", language="python")
                if text_of(msg.content).strip():
                    st.write(text_of(msg.content))
        elif isinstance(msg, ToolMessage):
            with st.chat_message("assistant"):
                _render_tool_result(msg)


class _Live:
    """Placeholders for the step being generated right now."""

    def __init__(self) -> None:
        self.new_step()

    def new_step(self) -> None:
        self.text_box = st.empty()
        self.calls_box = st.empty()

    def token(self, _piece: str, text: str) -> None:
        self.text_box.markdown(text + "▌")

    def partial_calls(self, partial: Dict[int, Dict[str, str]]) -> None:
        lines = [f"{c['name']}({c['args']})" for c in partial.values() if c["name"]]
        if lines:
            self.calls_box.code("\n".join(lines), language="python")

    def tool_calls(self, calls: List[Dict[str, Any]]) -> None:
        self.calls_box.code(
            "\n".join(f"{c['name']}({json.dumps(c['args'])})" for c in calls), language="python"
        )

    def finish(self, final_text: str) -> None:
        self.text_box.markdown(final_text if final_text else "")


def main() -> None:
    st.title("Wojtek x RAI" + (" (read-only)" if READ_ONLY else ""))
    graph = get_graph()
    with st.sidebar:
        st.subheader("Camera")
        st.session_state["show_depth"] = st.toggle("show depth", value=False)
        camera_panel()
    if "messages" not in st.session_state:
        st.session_state.messages = []
    st.chat_message("assistant").write(GREETING)
    _render_history([m for m in st.session_state.messages if not _is_system(m)])

    prompt = st.chat_input("Tell Wojtek what to do")
    if not prompt:
        return
    st.chat_message("user").write(prompt)
    history = [*st.session_state.messages, HumanMessage(content=prompt)]
    with st.chat_message("assistant"):
        live = _Live()
        events = TurnEvents(
            on_token=live.token,
            partial_calls=live.partial_calls,
            on_tool_calls=live.tool_calls,
            on_tool_result=_render_tool_result,
            on_step_end=live.new_step,
        )
        result = run_turn(graph, history, events)
        live.finish(result.final_text)
        # The Ollama tunnel flaps. When the model call dies after tools have
        # already moved the robot, run_turn still returns the completed
        # tool_call/ToolMessage pairs (with a placeholder AIMessage closing
        # the turn); keep them so the next turn knows what the robot did.
        error = getattr(result, "error", "")
        if error:
            st.error(f"LLM call failed: {error} -- the tool results above did run.")
    st.session_state.messages = result.messages


def _is_system(m: BaseMessage) -> bool:
    return getattr(m, "type", "") == "system"


if __name__ == "__main__":
    main()
