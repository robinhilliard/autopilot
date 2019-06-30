defmodule Autopilot.State do
  @moduledoc """
  Represent PIDs, setpoints, feedbacks and outputs of a fixed wing autopilot
  """
  
  alias XPlane.Data, as: Data
  alias XPlane.Cmd, as: Cmd
  
  
  defstruct [
    aircraft: nil,              # XPlane instance
    refresh_rate: 50,           # Hz
    time: :pos_infinity,        # simulation time, sec
    ap_mode: :off,              # :off, :flight_director, :on
    ap_airspeed_mode: :off,     # used to select airspeed rules
    ap_heading_mode: :off,      # used to select heading rules
    ap_altitude_mode: :off,     # used to select altitude rules
    phi_setpoint: 0.0,          # roll setpoint, degrees
    psi_setpoint: 0.0,          # heading setpoint, degrees
    theta_setpoint: 0.0,        # pitch setpoint, degrees
    mag_psi_setpoint: 0.0,      # selected magnetic heading
    altitude_setpoint: 0.0,     # selected altitude
    vs_setpoint: 0.0,           # vertical speed setpoint
    alpha_setpoint: 0.0,        # pitch angle of attack setpoint
    beta_setpoint: 0.0,        # yaw angle of attack setpoint
    airspeed_setpoint: 0.0,     # selected airspeed
    phi: 0.0,                   # roll, degrees
    psi: 0.0,                   # heading, degrees true, hpath + beta = psi
    theta: 0.0,                 # pitch, degrees
    vpath: 0.0,                 # pitch flown, degrees, vpath + alpha = theta
    alpha: 0.0,                 # pitch angle of attack, degrees
    hpath: 0.0,                 # track, degrees true
    beta: 0.0,                  # yaw angle of attack, degrees
    mag_psi: 0.0,               # magnetic heading
    groundspeed: 0.0,           # m/s
    height_msl: 0.0,            # m above sea level
    height_agl: 0.0,            # m above ground level
    latitude: 0.0,              # degrees
    longitude: 0.0,             # degrees
    true_airspeed: 0.0,         # TAS, not indicated, do not fly to this, kts
    indicated_airspeed: 0.0,    # IAS, air density accounted for, kts
    mass_total: 0.0,            # total weight, kgs
    aileron_trim: 0.0,          # -1.0 ... 1.0 left to right output
    elevator_trim: 0.0,         # -1.0 ... 1.0 down to up output
    rudder_trim: 0.0            # -1.0 ... 1.0 left to right output
  ]
  
  @type t :: %__MODULE__{
    aircraft: %XPlane.Instance{},
    refresh_rate: integer,
    time: atom | float,
    ap_mode: atom,
    ap_airspeed_mode: atom,
    ap_heading_mode: atom,
    ap_altitude_mode: atom,
    phi_setpoint: float,
    psi_setpoint: float,
    theta_setpoint: float,
    mag_psi_setpoint: float,
    altitude_setpoint: float,
    vs_setpoint: float,
    alpha_setpoint: float,
    beta_setpoint: float,
    airspeed_setpoint: float,
    phi: float,
    psi: float,
    theta: float,
    vpath: float,
    alpha: float,
    hpath: float,
    beta: float,
    mag_psi: float,
    groundspeed: float,
    height_msl: float,
    height_agl: float,
    latitude: float,
    longitude: float,
    true_airspeed: float,
    indicated_airspeed: float,
    mass_total: float,
    aileron_trim: float,
    elevator_trim: float,
    rudder_trim: float
  }

  
  @spec init(list({atom, any})) :: Autopilot.State.t
  def init(params \\ []) do
    struct(%__MODULE__{}, params)
    |> subscribe_to_feedback()
  end
  
  
  @spec update_feedback(Autopilot.State.t) :: Autopilot.State.t
  def update_feedback(state = %__MODULE__{aircraft: aircraft}) do
    feedback = Data.latest_updates(aircraft, [
      :time_total_flight_time_sec,
      :cockpit_autopilot_heading_mag,
      :flightmodel_position_true_phi,
      :flightmodel_position_true_psi,
      :flightmodel_position_true_theta,
      :flightmodel_position_vpath,
      :flightmodel_position_alpha,
      :flightmodel_position_hpath,
      :flightmodel_position_beta,
      :flightmodel_position_mag_psi,
      :flightmodel_position_groundspeed,
      :flightmodel_position_elevation,
      :flightmodel_position_y_agl,
      :flightmodel_position_latitude,
      :flightmodel_position_longitude,
      :flightmodel_position_true_airspeed,
      :flightmodel_position_indicated_airspeed,
      :flightmodel_weight_m_total
    ])
    %__MODULE__{ state |
      time: feedback[:time_total_flight_time_sec],
      mag_psi_setpoint: feedback[:cockpit_autopilot_heading_mag],
      phi: feedback[:flightmodel_position_true_phi],
      psi: feedback[:flightmodel_position_true_psi],
      theta: feedback[:flightmodel_position_true_theta],
      vpath: feedback[:flightmodel_position_vpath],
      alpha: feedback[:flightmodel_position_alpha],
      hpath: feedback[:flightmodel_position_hpath],
      beta: feedback[:flightmodel_position_beta],
      mag_psi: feedback[:flightmodel_position_mag_psi],
      groundspeed: feedback[:flightmodel_position_groundspeed],
      height_msl: feedback[:flightmodel_position_elevation],
      height_agl: feedback[:flightmodel_position_y_agl],
      latitude: feedback[:flightmodel_position_latitude],
      longitude: feedback[:flightmodel_position_longitude],
      true_airspeed: feedback[:flightmodel_position_true_airspeed],
      indicated_airspeed: feedback[:flightmodel_position_indicated_airspeed],
      mass_total: feedback[:flightmodel_position_m_total]
    }
  end
  
  
  @spec set_output(Autopilot.State.t, list(atom)) :: Autopilot.State.t
  def set_output(state = %__MODULE__{aircraft: aircraft},
        refs \\ [:aileron_trim, :elevator_trim, :rudder_trim]) do
    :ok = aircraft
          |> Data.set(for ref <- refs, do: {to_xplane(ref, aircraft.version_number), Map.fetch!(state, ref)})
    state
  end
  
  
  @spec subscribe_to_feedback(Autopilot.State.t) :: Autopilot.State.t
  defp subscribe_to_feedback(state = %__MODULE__{aircraft: aircraft, refresh_rate: rate}) do
    :ok = Data.request_updates(aircraft, [
      time_total_flight_time_sec: rate,
      cockpit_autopilot_heading_mag: rate,
      flightmodel_position_true_phi: rate,
      flightmodel_position_true_psi: rate,
      flightmodel_position_true_theta: rate,
      flightmodel_position_vpath: rate,
      flightmodel_position_alpha: rate,
      flightmodel_position_hpath: rate,
      flightmodel_position_beta: rate,
      flightmodel_position_mag_psi: rate,
      flightmodel_position_groundspeed: rate,
      flightmodel_position_elevation: rate,
      flightmodel_position_y_agl: rate,
      flightmodel_position_latitude: rate,
      flightmodel_position_longitude: rate,
      flightmodel_position_true_airspeed: rate,
      flightmodel_position_indicated_airspeed: rate,
      flightmodel_weight_m_total: rate
    ])
    state
  end
  
  defp to_xplane(:aileron_trim, version) when version >= 110000, do: :flightmodel_controls_ail_trim
  defp to_xplane(:elevator_trim, version) when version >= 110000, do: :flightmodel_controls_elv_trim
  defp to_xplane(:rudder_trim, version) when version >= 110000, do: :flightmodel_controls_rud_trim
  
end
