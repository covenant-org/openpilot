#!/usr/bin/env python3
import argparse
import asyncio
import logging

import aiohttp

import aiortc
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, RTCIceServer, RTCConfiguration
from openpilot.system.webrtc.device.video import LiveStreamVideoStreamTrack
from aiortc.contrib.media import MediaPlayer
from threading import Event

pcs = set()
_exit = Event()


class WhipSession:
    def __init__(self, url):
        self._http = None
        self._whip_url = url
        self._session_url = None
        self._turn = None
        self._offersdp = None
        self._answersdp = None

    async def createEndpoint(self, offer):
        self._http = aiohttp.ClientSession(
            connector=aiohttp.TCPConnector(ssl=False))
        self._offersdp = offer.sdp
        headers = {'content-type': 'application/sdp'}
        async with self._http.post(self._whip_url, headers=headers, data=self._offersdp) as response:
            print(response)
            location = response.headers["Location"]
            assert isinstance(location, str)
            self._answersdp = await response.text()
            host = self._whip_url.split("//")[-1].split("/")[0]
            self._session_url = "http://" + host + location

    async def trickle(self, data):
        self._http = aiohttp.ClientSession()
        headers = {'content-type': 'application/trickle-ice-sdpfrag',
                   'Authorization': 'Bearer +' + self._token}
        async with self._http.patch(self._session_url, headers=headers, data=data) as response:
            print(response)
            data = await response.text()

    async def destroy(self):
        if self._session_url:
            headers = {}
            async with self._http.delete(self._session_url, headers=headers) as response:
                print(response)
                assert response.ok == True
            self._session_url = None

        if self._http:
            await self._http.close()
            self._http = None


async def publish(session, track):
    """
    Live stream video to the room.
    """
    if session._turn:
        turnArr = session._turn.split("@")[0].split(":")
        turnurl = turnArr[0] + ":" + turnArr[1] + ":" + turnArr[2]
        turnuser = session._turn.split("@")[1].split(":")[0]
        turnpass = session._turn.split("@")[1].split(":")[1]
        pc = RTCPeerConnection(configuration=RTCConfiguration(
            iceServers=[RTCIceServer(
                urls=["stun:stun.l.google.com:19302", turnurl],
                username=turnuser,
                credential=turnpass)]))
    else:
        pc = RTCPeerConnection(configuration=RTCConfiguration(
            iceServers=[RTCIceServer("stun:stun.l.google.com:19302")]))

    pcs.add(pc)
    pc.addTransceiver("video", direction="sendonly")
#    pc.addTransceiver("audio", direction="sendonly")

    @pc.on("iceconnectionstatechange")
    async def on_iceconnectionstatechange():
        print("ICE connection state is", pc.iceConnectionState)
        if pc.iceConnectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    # configure media
    sender = pc.addTrack(track)
    if hasattr(track, "codec_preference") and track.codec_preference() is not None:
      transceiver = next(t for t in pc.getTransceivers() if t.sender == sender)
      codec_mime = f"video/{track.codec_preference().upper()}"
      rtp_codecs = aiortc.RTCRtpSender.getCapabilities("video").codecs
      rtp_codec = [c for c in rtp_codecs if c.mimeType == codec_mime]
      transceiver.setCodecPreferences(rtp_codec)

    # send offer
    offer = await pc.createOffer()
    await pc.setLocalDescription(offer)

    # Create WHIP endpoint with the SDP offer
    await session.createEndpoint(offer)
    print(session)
    # apply answer
    await pc.setRemoteDescription(
        RTCSessionDescription(
            sdp=session._answersdp, type="answer"
        )
    )
    # aiortc doesn't support trickle ice yet
    # await session.trickle("")


async def run(track, session):
    # send video
    frame = await track.recv()
    await publish(session=session, track=track)
    # exchange media for 1 minute
    print("Exchanging media...")
    while not _exit.is_set():
        await asyncio.sleep(600)
    print("Done")


if __name__ == "__main__":
    loop = asyncio.get_event_loop()

    track = LiveStreamVideoStreamTrack("road")

    # HTTP signaling and peer connection
    session = WhipSession("http://159.54.131.60:8889/comma/whip")

    # create media source

    try:
        loop.run_until_complete(
            run(track=track, session=session)
        )
    except KeyboardInterrupt:
        _exit.set()
    finally:
        loop.run_until_complete(session.destroy())
        # close peer connections
        coros = [pc.close() for pc in pcs]
        loop.run_until_complete(asyncio.gather(*coros))
