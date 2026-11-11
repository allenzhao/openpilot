"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base import BrandSettings
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp

TORQUE_INTERCEPTOR_DESC = tr_noop(
  "Steer through a Torque Interceptor, down to walking speed. Turn on only if one is installed between the steering " +
  "torque sensor and the EPS and connected to the OBD-II cable. With this on and no interceptor answering, openpilot " +
  "will not engage. Takes effect on the next drive."
)


class MazdaSettings(BrandSettings):
  def __init__(self):
    super().__init__()
    # card reads the param when a drive starts, so it is set between drives: no restart to ask for
    self.torque_interceptor_toggle = toggle_item_sp(tr("Torque Interceptor"), "", param="TorqueInterceptorEnabled")
    self.items = [self.torque_interceptor_toggle]

  def update_settings(self):
    offroad = ui_state.is_offroad()
    desc = tr(TORQUE_INTERCEPTOR_DESC)
    if not offroad:
      disabled_msg = tr("Enable \"Always Offroad\" in Device panel, or turn vehicle off to toggle.")
      desc = f"<b>{disabled_msg}</b><br><br>{desc}"
    self.torque_interceptor_toggle.set_description(desc)
    self.torque_interceptor_toggle.action_item.set_enabled(offroad)
