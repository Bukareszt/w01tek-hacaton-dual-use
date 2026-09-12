"""Waiting for the next MJPEG frame without losing the viewer.

aiohttp does not cancel a handler when its client goes away. A stream handler
parked on an empty queue never writes, so it never sees the error a write to
a gone client raises, and it stays parked for as long as the camera stays
silent. That is the normal case for the tower stream on a robot without the
tower camera: the page opens on it, gives up after a few seconds and switches
to the front camera, and the tower handler would sit there for good, holding
its viewer queue and the tower subscription with it.

So the wait is bounded. Every `poll_s` the handler looks at the transport,
and a closed one ends the stream the way a failed write would. Pure asyncio
on purpose, so the rule has a model-free unit test.
"""

import asyncio

STREAM_POLL_S = 2.0   # how long a viewer with no frames goes unchecked


def viewer_gone(request):
    """Whether the client behind `request` has closed its connection."""
    transport = getattr(request, "transport", None)
    return transport is None or transport.is_closing()


async def next_frame(q, request, poll_s=STREAM_POLL_S):
    """The next JPEG from `q`, or None once the viewer is gone.

    A frame is returned as soon as one arrives. With none, the transport is
    checked every `poll_s`; a viewer that has left is reported by None, and
    the handler's cleanup then runs without a frame ever having arrived.
    """
    while True:
        try:
            return await asyncio.wait_for(q.get(), timeout=poll_s)
        except asyncio.TimeoutError:
            if viewer_gone(request):
                return None
