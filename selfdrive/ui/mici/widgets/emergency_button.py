"""The driver's panic button.

Pressed when something is wrong that only the driver can see — a hijacking, a
crash, a medical emergency. The press has to reach a human who can act, so the
only jobs here are: be unmistakable, be hittable without reading anything, and
never silently fail.

HOW THE PRESS LEAVES THIS PROCESS. The UI is a pure subscriber to cereal; it
has no link to Supabase and no business growing one. The covenant daemon
already owns a door for exactly this — `/data/covenant/manual_events/`, a drop
directory the clipper drains every 2 s — built so "a detector in another
process can fire a real, enriched event through this door instead of growing
its own copy of the clip pipeline". A press writes one JSON file there and is
done. The clipper then fires event code 13 (`sos`, critical), cuts the clip,
and the existing EVENT frame → persister → `drive_event` path carries it, where
galleon's `drive_event_notify_critical` trigger fires on severity alone.

THE WRITE IS ATOMIC, and that is not decoration. The drainer lists `*.json` and
parses whatever it finds, rejecting anything it cannot read. A half-written
file would be a rejected emergency. So the payload is written under a `.tmp`
name the drainer ignores and then renamed, which on the same filesystem is
atomic — the file does not exist until it is complete.

WHY IT CANNOT RAISE. This runs on the render thread. An exception here would
take the whole cab screen down at the moment the driver needed it, so every
failure is caught and surfaced on the button instead.

IT FIRES ON A HOLD, NOT A TAP, because of where this screen sits. It is the
screen the truck rests on, so it is what is under anyone's hand when the cab is
parked — someone cleaning the glass, getting in, a knee, a clipboard. A tap
would page four people at ntfy priority 5, which bypasses do-not-disturb. A
handful of those is all it takes to teach everyone that this is the alert you
swipe away, and then the real press arrives to an audience that has already
learned to ignore it. The alarm's value is entirely in people still believing
it.

A hold costs the driver about a second and a half and cannot happen by
accident. It is still ONE gesture with nothing to read, which is the property
that matters when someone is frightened, and the ring filling around the button
is its own instruction: nobody needs to be told what a filling circle means.
"""
import json
import os
import time
import uuid
from datetime import datetime, timezone

import pyray as rl

from openpilot.selfdrive.ui.mici.widgets.button import BigCircleButton
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.label import Label

MANUAL_EVENT_DIR = os.getenv('MANUAL_EVENT_DIR', '/data/covenant/manual_events')
EVENT_TYPE = 'sos'

BUTTON_SIZE = 180
CONFIRM_HOLD_S = 4.0   # how long the button reports the outcome before resetting

# How long the driver must hold. Long enough that a brush or a knee cannot reach
# it, short enough that it is not an obstacle to someone in trouble. The ring
# makes the wait legible, which is what stops it feeling like an unresponsive
# button.
HOLD_TO_FIRE_S = 1.5

# The progress ring, drawn just inside the circle's edge.
RING_INNER_R = 78.0
RING_OUTER_R = 87.0
RING_SEGMENTS = 64
RING_COLOR = rl.Color(255, 255, 255, 230)

# The red circle carries the alarm; the face only has to stay legible on it, so
# it is white at full opacity rather than the 0.9 used for ordinary labels.
FACE_COLOR = rl.Color(255, 255, 255, 255)


def write_sos_request(directory: str = MANUAL_EVENT_DIR,
                      at: datetime | None = None) -> str:
    """Drop one manual-event request for the daemon. Returns the path written.

    Raises on failure — the caller decides what the driver is told. Kept free
    of raylib so it can be tested without a window.
    """
    at = at or datetime.now(timezone.utc)
    os.makedirs(directory, exist_ok=True)
    payload = {
        'type': EVENT_TYPE,
        'value': 1.0,
        'at': at.isoformat().replace('+00:00', 'Z'),
        # `summary` rides through to drive_event.summary, which is what the
        # notification body is built from. Saying where it came from matters:
        # every other critical event is a detector's inference, and a reader
        # needs to know this one is a person.
        'summary': {'source': 'cab_ui', 'trigger': 'driver_press'},
    }
    name = f'{EVENT_TYPE}-{int(at.timestamp())}-{uuid.uuid4().hex[:8]}'
    tmp = os.path.join(directory, name + '.tmp')
    final = os.path.join(directory, name + '.json')
    with open(tmp, 'w') as f:
        json.dump(payload, f)
        f.flush()
        os.fsync(f.fileno())      # survive a power cut between press and drain
    os.replace(tmp, final)
    return final


class EmergencyButton(BigCircleButton):
  """A red `BigCircleButton` whose face is a word, and which reports its own
  outcome.

  Subclassing rather than composing keeps the press behaviour — the bounce
  scale, the 75 ms click delay, the pressed-texture swap — identical to every
  other button on the device, so the panic button does not feel like a
  different piece of software at the moment it is used.
  """

  def __init__(self, variant: str = "sos"):
    icon = gui_app.texture("icons_mici/exclamation_point.png", 22, 113) if variant == "icon" else None
    super().__init__(icon, red=True)

    self._variant = variant
    self._state = 'idle'          # idle | sent | failed
    self._state_until = 0.0
    # None when not holding; otherwise the monotonic time the hold began.
    self._hold_start: float | None = None
    self._on_fire = None
    self._labels = {
      'sos':  Label("SOS", font_size=72, font_weight=FontWeight.BOLD, text_color=FACE_COLOR),
      'text': Label("EMERGENCIA", font_size=22, font_weight=FontWeight.BOLD, text_color=FACE_COLOR),
      # Confirmation has to be readable at a glance by someone who is not calm.
      'sent':   Label("ENVIADO", font_size=34, font_weight=FontWeight.BOLD, text_color=FACE_COLOR),
      'failed': Label("REINTENTAR", font_size=24, font_weight=FontWeight.BOLD, text_color=FACE_COLOR),
    }

  def press(self) -> bool:
    """Write the request and record the outcome for the face. Never raises."""
    try:
      path = write_sos_request()
      self._state = 'sent'
      print(f"SOS written: {path}", flush=True)
      ok = True
    except Exception as exc:
      # The driver is told to try again rather than being left with a button
      # that looked like it worked. Silence here is the worst outcome of all.
      self._state = 'failed'
      print(f"SOS FAILED: {exc}", flush=True)
      ok = False
    self._state_until = time.monotonic() + CONFIRM_HOLD_S
    return ok

  def set_fire_callback(self, cb) -> None:
    self._on_fire = cb

  def hold_progress(self) -> float:
    """0..1 of the way to firing. 0 when not held."""
    if self._hold_start is None:
      return 0.0
    return min(1.0, (time.monotonic() - self._hold_start) / HOLD_TO_FIRE_S)

  def _update_state(self) -> None:
    now = time.monotonic()
    if self._state != 'idle' and now >= self._state_until:
      self._state = 'idle'

    # The hold is driven from `is_pressed` rather than from press/release
    # events, because that is the flag the base Widget already maintains for
    # "the finger is down AND still inside this rect". Sliding off the button
    # clears it, which is exactly the escape hatch someone needs when they
    # realise they have started an alarm they did not mean to start.
    if self._state == 'idle' and self.is_pressed:
      if self._hold_start is None:
        self._hold_start = now
      elif now - self._hold_start >= HOLD_TO_FIRE_S:
        self._hold_start = None
        (self._on_fire or self.press)()
    elif not self.is_pressed:
      self._hold_start = None

  def _face(self) -> Label | None:
    if self._state != 'idle':
      return self._labels[self._state]
    return self._labels.get(self._variant)

  def _draw_ring(self, btn_y: float) -> None:
    progress = self.hold_progress()
    if progress <= 0.0:
      return
    # `_draw_content` is handed only the ANIMATED top edge, but the button is
    # square and the press animation scales it about its centre, so the
    # horizontal inset equals the vertical one and the scale can be recovered
    # from that single number. That keeps the ring locked to the circle through
    # the bounce instead of floating over a shrinking button.
    inset = btn_y - self._rect.y
    scale = 1.0 - (2.0 * inset / self._rect.height)
    cx = self._rect.x + inset + (self._rect.width * scale) / 2.0
    cy = btn_y + (self._rect.height * scale) / 2.0
    # Clockwise from 12 o'clock: the direction every progress dial turns.
    rl.draw_ring(rl.Vector2(cx, cy), RING_INNER_R * scale, RING_OUTER_R * scale,
                 -90.0, -90.0 + 360.0 * progress, RING_SEGMENTS, RING_COLOR)

  def _draw_content(self, btn_y: float) -> None:
    self._draw_ring(btn_y)
    face = self._face()
    if face is None:
      super()._draw_content(btn_y)
      return
    # `_draw_content` is handed the ANIMATED top edge, so drawing against it
    # makes the face ride the press bounce instead of sitting still inside a
    # moving circle.
    face.render(rl.Rectangle(self._rect.x, btn_y, self._rect.width, self._rect.height))


class EmergencyScreen(Widget):
  """Owns the canvas and centres the button in it.

  `gui_app.render()` renders every nav-stack widget with
  `widget.render(Rectangle(0, 0, width, height))`, and `Widget.render` adopts
  whatever rect it is handed — so a button pushed straight onto the stack is
  silently resized to the whole canvas and its own geometry is thrown away.
  The screen takes the canvas; the button keeps its 180x180.
  """

  def __init__(self, variant: str = "sos", on_press=None):
    super().__init__()
    self._button = EmergencyButton(variant)
    # Deliberately NOT set_click_callback: a click is a release, and a release
    # is what we are refusing to fire on. The button calls this itself once the
    # hold completes.
    self._button.set_fire_callback(on_press or self._button.press)

  @property
  def button(self) -> EmergencyButton:
    return self._button

  def _render(self, rect: rl.Rectangle) -> None:
    self._button.render(rl.Rectangle(rect.x + (rect.width - BUTTON_SIZE) / 2,
                                     rect.y + (rect.height - BUTTON_SIZE) / 2,
                                     BUTTON_SIZE, BUTTON_SIZE))


if __name__ == "__main__":
  gui_app.init_window("emergency")
  gui_app.push_widget(EmergencyScreen(os.getenv("VARIANT", "sos")))
  for _ in gui_app.render():
    pass
  gui_app.close()
