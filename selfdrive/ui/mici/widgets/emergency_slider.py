"""Centre-out slide: left reports a breakdown, right calls for help.

WHY A SLIDE AND NOT A TAP OR A HOLD. This is the screen the truck rests on, so
it is under anyone's hand whenever the cab is parked — a tap would page people
because someone cleaned the glass. A 1.5 s hold solved that and felt like the
device was ignoring you, which is its own failure: the gesture you make when
something is wrong should not feel unresponsive. A slide is deliberate by
construction and finishes the instant you let go, so it needs no artificial
delay to prove you meant it.

WHY TWO DIRECTIONS FROM THE CENTRE. The driver has exactly two things to say
and no time to choose from a menu: the truck is broken, or I am in trouble.
Putting them on one handle means there is nothing to read, nothing to aim at,
and the choice is a direction rather than a target.

This is comma's own slide-to-confirm, rebuilt for two directions. Theirs
(`system/ui/widgets/slider.py`) anchors the handle at the right edge and only
travels left — `_drag_threshold = -width // 2` — so it cannot express a
centre-out choice. The textures, the filter time constants and the press bounce
are taken from it deliberately, so this feels like the rest of the device.
"""
import json
import os
import time
import uuid
from datetime import datetime, timezone

import pyray as rl

from openpilot.common.filter_simple import BounceFilter, FirstOrderFilter
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.label import Label

MANUAL_EVENT_DIR = os.getenv('MANUAL_EVENT_DIR', '/data/covenant/manual_events')
SOS = 'sos'
MECHANICAL = 'mechanical'

TRACK_W, TRACK_H = 520, 180
HANDLE = 180

# How far the handle may travel each way, and how far it must go to count.
# Travel is what the track geometry allows; the threshold is 60% of it —
# far enough that a knock cannot reach it, short enough that a frightened
# person completes it first time.
TRAVEL = (TRACK_W - HANDLE) // 2          # 170 px
THRESHOLD = int(TRAVEL * 0.6)             # 102 px

# Label column at each end. Wider than the handle's travel so a long word is
# not squeezed into the gap the handle leaves; it overlaps the handle instead,
# and is drawn on top of it.
LABEL_W = 210

PRESSED_SCALE = 1.07
RETURN_RC = 0.05        # same as comma's slider: snappy, not instant
CONFIRM_HOLD_S = 4.0    # how long the outcome stays on screen

LEFT, RIGHT = -1, 1


class EmergencySlider(Widget):
  """A red handle resting at centre. Drag past the threshold either way."""

  def __init__(self, left_label: str, right_label: str,
               on_left=None, on_right=None):
    super().__init__()
    self._on = {LEFT: on_left, RIGHT: on_right}

    self._bg = gui_app.texture("icons_mici/buttons/slider_bg.png", TRACK_W, TRACK_H)
    self._circle = gui_app.texture("icons_mici/buttons/button_circle_red.png", HANDLE, HANDLE)
    self._circle_pressed = gui_app.texture("icons_mici/buttons/button_circle_red_pressed.png", HANDLE, HANDLE)

    self._labels = {
      LEFT: Label(left_label, font_size=30, font_weight=FontWeight.BOLD, text_color=rl.WHITE),
      RIGHT: Label(right_label, font_size=30, font_weight=FontWeight.BOLD, text_color=rl.WHITE),
    }
    self._result_label = Label("", font_size=34, font_weight=FontWeight.BOLD, text_color=rl.WHITE)

    self._offset = 0.0                    # live drag offset from centre, px
    self._offset_filter = FirstOrderFilter(0.0, RETURN_RC, 1 / gui_app.target_fps)
    self._scale_filter = BounceFilter(1.0, 0.1, 1 / gui_app.target_fps)
    self._dragging = False
    self._grab_x = 0.0
    self._state = 'idle'                  # idle | sent | failed
    self._state_until = 0.0

  # ── geometry ──────────────────────────────────────────────────────────────
  def _track_x(self) -> float:
    return self._rect.x + (self._rect.width - TRACK_W) / 2

  def _track_y(self) -> float:
    return self._rect.y + (self._rect.height - TRACK_H) / 2

  def _handle_x(self) -> float:
    centre = self._track_x() + (TRACK_W - HANDLE) / 2
    return centre + self._offset_filter.x

  def direction(self) -> int:
    """Which way the handle currently leans past the threshold, else 0."""
    if self._offset_filter.x <= -THRESHOLD:
      return LEFT
    if self._offset_filter.x >= THRESHOLD:
      return RIGHT
    return 0

  def progress(self, side: int) -> float:
    """0..1 of the way to firing on `side`. Drives the label reveal."""
    v = self._offset_filter.x * side
    return min(max(v / THRESHOLD, 0.0), 1.0)

  # ── interaction ───────────────────────────────────────────────────────────
  def _handle_mouse_event(self, mouse_event) -> None:
    super()._handle_mouse_event(mouse_event)
    if self._state != 'idle':
      return

    if mouse_event.left_pressed:
      hit = rl.Rectangle(self._handle_x(), self._track_y(), HANDLE, TRACK_H)
      if rl.check_collision_point_rec(mouse_event.pos, hit):
        self._dragging = True
        self._grab_x = mouse_event.pos.x - self._offset

    elif mouse_event.left_released and self._dragging:
      self._dragging = False
      side = self.direction()
      # Fires on RELEASE, not on crossing: the driver can still back out by
      # sliding back to centre, which is the only undo a single gesture can
      # offer.
      if side:
        self.fire(side)
      self._offset = 0.0

    elif self._dragging:
      self._offset = max(-TRAVEL, min(TRAVEL, mouse_event.pos.x - self._grab_x))

  def fire(self, side: int) -> None:
    """Run the side's callback. Never raises — this is the render thread."""
    cb = self._on.get(side)
    try:
      ok = True if cb is None else cb()
      self._state = 'sent' if ok is not False else 'failed'
    except Exception as exc:                      # noqa: BLE001 — see above
      print(f"emergency slider callback failed: {exc}", flush=True)
      self._state = 'failed'
    self._result_label.set_text('ENVIADO' if self._state == 'sent' else 'REINTENTAR')
    self._state_until = time.monotonic() + CONFIRM_HOLD_S

  def _update_state(self) -> None:
    if self._state != 'idle' and time.monotonic() >= self._state_until:
      self._state = 'idle'
      self._result_label.set_text('')
    # 1:1 while the finger is down; filtered on the way home so the handle
    # settles instead of snapping.
    if self._dragging:
      self._offset_filter.x = self._offset
    else:
      self._offset_filter.update(self._offset)

  # ── drawing ───────────────────────────────────────────────────────────────
  def _render(self, _rect) -> None:
    tx, ty = self._track_x(), self._track_y()
    rl.draw_texture_ex(self._bg, rl.Vector2(tx, ty), 0.0, 1.0, rl.WHITE)

    if self._state != 'idle':
      self._result_label.render(rl.Rectangle(tx, ty, TRACK_W, TRACK_H))
      return

    pressed = self._dragging
    scale = self._scale_filter.update(PRESSED_SCALE if pressed else 1.0)
    tex = self._circle_pressed if pressed else self._circle
    hx = self._handle_x() + (HANDLE * (1 - scale)) / 2
    hy = ty + (TRACK_H - HANDLE * scale) / 2
    rl.draw_texture_ex(tex, rl.Vector2(hx, hy), 0.0, scale, rl.WHITE)

    # LABELS ON TOP OF THE HANDLE, not under it. They sit at the ends of the
    # track, which is exactly where the handle travels to — drawn underneath,
    # the word you are confirming disappears beneath the thing confirming it,
    # right at the moment you need to be sure you picked the correct side.
    # White reads on both the black track and the red handle, so one colour
    # works the whole way across.
    for side in (LEFT, RIGHT):
      alpha = int(255 * (0.45 + 0.55 * self.progress(side)))
      label = self._labels[side]
      label.set_text_color(rl.Color(255, 255, 255, alpha))
      edge = tx if side == LEFT else tx + TRACK_W - LABEL_W
      label.render(rl.Rectangle(edge, ty, LABEL_W, TRACK_H))


def write_manual_event(event_type: str, directory: str = MANUAL_EVENT_DIR,
                       at: datetime | None = None) -> str:
    """Drop one manual-event request for the daemon. Returns the path written.

    HOW THE GESTURE LEAVES THIS PROCESS. The UI is a pure cereal subscriber with
    no link to Supabase and no business growing one. The covenant daemon already
    owns a door built for exactly this — a drop directory its clipper drains
    every 2 s — so a slide writes one JSON file and is done. The clipper fires
    the event, cuts the clip, and the existing frame -> persister -> drive_event
    path carries it.

    THE WRITE IS ATOMIC, and that is not decoration. The drainer lists `*.json`
    and parses whatever it finds, rejecting what it cannot read. A half-written
    file would be a rejected emergency. So the payload goes down under a `.tmp`
    name the drainer ignores and is then renamed, which on one filesystem is
    atomic: the file does not exist until it is complete.

    Raises on failure — the caller decides what the driver is told.
    """
    at = at or datetime.now(timezone.utc)
    os.makedirs(directory, exist_ok=True)
    payload = {
        'type': event_type,
        'value': 1.0,
        'at': at.isoformat().replace('+00:00', 'Z'),
        # Rides through to drive_event.summary, which is what the notification
        # body is built from. Saying where it came from matters: every other
        # critical event is a detector's inference, and a reader needs to know
        # this one is a person.
        'summary': {'source': 'cab_ui', 'trigger': 'driver_slide'},
    }
    name = f'{event_type}-{int(at.timestamp())}-{uuid.uuid4().hex[:8]}'
    tmp = os.path.join(directory, name + '.tmp')
    final = os.path.join(directory, name + '.json')
    with open(tmp, 'w') as f:
        json.dump(payload, f)
        f.flush()
        os.fsync(f.fileno())      # survive a power cut between slide and drain
    os.replace(tmp, final)
    return final


class EmergencyScreen(Widget):
  """Owns the canvas and centres the slider in it.

  `gui_app.render()` renders every nav-stack widget with
  `widget.render(Rectangle(0, 0, width, height))`, and `Widget.render` adopts
  whatever rect it is handed — so a control pushed straight onto the stack is
  silently resized to the whole canvas. The screen takes the canvas; the slider
  lays itself out inside whatever it is given.
  """

  def __init__(self, on_fire=None):
    super().__init__()
    self._on_fire = on_fire or self._report
    self._slider = EmergencySlider('MECÁNICA', 'SOS',
                                   on_left=lambda: self._on_fire(MECHANICAL),
                                   on_right=lambda: self._on_fire(SOS))

  @property
  def slider(self) -> EmergencySlider:
    return self._slider

  @staticmethod
  def _report(event_type: str) -> bool:
    """Write the request. Never raises — this is the render thread, and an
    exception here would take the cab screen down at the moment it is needed."""
    try:
      path = write_manual_event(event_type)
      print(f'{event_type} written: {path}', flush=True)
      return True
    except Exception as exc:                       # noqa: BLE001 — see above
      print(f'{event_type} FAILED: {exc}', flush=True)
      return False

  def _render(self, rect: rl.Rectangle) -> None:
    self._slider.render(rect)


if __name__ == '__main__':
  gui_app.init_window('emergency')
  gui_app.push_widget(EmergencyScreen())
  for _ in gui_app.render():
    pass
  gui_app.close()
