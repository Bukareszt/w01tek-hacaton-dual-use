"""Model-free tests of the bounded frame wait behind /stream.mjpg."""
import asyncio

from wojtek_deck.stream import next_frame, viewer_gone


class FakeTransport:
    def __init__(self, closing=False):
        self.closing = closing

    def is_closing(self):
        return self.closing


class FakeRequest:
    def __init__(self, transport):
        self.transport = transport


def run(coro):
    return asyncio.new_event_loop().run_until_complete(coro)


def test_a_frame_is_returned_as_soon_as_it_arrives():
    async def go():
        q = asyncio.Queue(maxsize=1)
        q.put_nowait(b"jpeg")
        return await next_frame(q, FakeRequest(FakeTransport()), poll_s=5.0)
    assert run(go()) == b"jpeg"


def test_a_viewer_that_left_is_found_without_a_frame():
    # The tower stream on a robot with no tower camera: nothing is ever
    # queued. The page gives up and closes; the handler must find out.
    async def go():
        q = asyncio.Queue(maxsize=1)
        request = FakeRequest(FakeTransport(closing=True))
        return await asyncio.wait_for(
            next_frame(q, request, poll_s=0.01), timeout=1.0)
    assert run(go()) is None


def test_a_connection_lost_transport_counts_as_gone():
    # aiohttp drops the transport reference once the connection is lost.
    assert viewer_gone(FakeRequest(None)) is True
    assert viewer_gone(FakeRequest(FakeTransport())) is False


def test_a_waiting_viewer_is_kept_while_its_connection_is_open():
    # No frame yet, connection still open: keep waiting past the poll and
    # hand over the frame when it comes.
    async def go():
        q = asyncio.Queue(maxsize=1)
        request = FakeRequest(FakeTransport())
        task = asyncio.ensure_future(next_frame(q, request, poll_s=0.01))
        await asyncio.sleep(0.05)      # several polls with nothing to give
        assert not task.done()
        q.put_nowait(b"late")
        return await asyncio.wait_for(task, timeout=1.0)
    assert run(go()) == b"late"
