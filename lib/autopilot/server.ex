defmodule Autopilot.Server do
  @moduledoc """
  GenServer that controls a specific aircraft
  """
  
  use GenServer
  
  alias Autopilot.State, as: State
  alias Autopilot.PIDController, as: PID


  
  def start_link(instance, opts \\ []) do
    GenServer.start_link(__MODULE__, instance, [name: name(instance)] ++ opts)
  end
  
  
  def get_state(pid) do
    GenServer.call(pid, :get_state)
  end

  
  @impl true
  def init(aircraft) do
    schedule_update()
    {
      :ok,
      State.init(aircraft: aircraft, ap_mode: :on, ap_heading_mode: :on)
      |> init_airspeed_mode
      |> init_heading_mode
      |> init_altitude_mode
    }
  end

  
  @impl true
  def handle_call(_msg, _from, state) do
    {:reply, state, state}
  end

  
  @impl true
  def handle_cast(_msg, state) do
    {:noreply, state}
  end
  
  
  @impl true
  def handle_info(:check, state = %State{ap_mode: :off}) do
    schedule_update()
    state
  end
  
  def handle_info(:check, state = %State{ap_mode: :on}) do
    schedule_update()
    {
      :noreply,
      state
      |> State.update_feedback
      |> do_airspeed_mode
      |> do_heading_mode
      |> do_altitude_mode
    }
  end
  
  defp schedule_update do
    Process.send_after(self(), :check, 20)
  end
  
  
  defp name(instance) do
    String.to_atom("#{__MODULE__}_#{instance.addr}")
  end
  
  
  # AIRSPEED
  
  defp init_airspeed_mode(state) do
    state
  end
  
  
  defp do_airspeed_mode(state = %State{ap_airspeed_mode: :off}) do
    state
  end
  
  
  # HEADING
  
  defp init_heading_mode(state) do
    state
    |> PID.add_pid(:mag_psi, :mag_psi_setpoint, :phi_setpoint,
         p: 1.0, d: -0.5, output_min: -30.0, output_max: 30.0, modulo: 360.0)
    |> PID.add_pid(:phi, :phi_setpoint, :aileron_trim,
         p: 0.5, output_min: -1.0, output_max: 1.0)
    |> PID.add_pid(:beta, :beta_setpoint, :rudder_trim, p: 1.0)
  end
  
  
  defp do_heading_mode(state = %State{ap_heading_mode: :off}) do
    state
  end
  
  defp do_heading_mode(state = %State{ap_heading_mode: :on}) do
    state
    |> PID.set_pid_output(:mag_psi, :mag_psi_setpoint, :phi_setpoint)
    |> PID.set_pid_output(:phi, :phi_setpoint, :aileron_trim)
    |> PID.set_pid_output(:beta, :beta_setpoint, :rudder_trim)
    |> State.set_output([:aileron_trim, :rudder_trim])
  end
  
  
  # ALTITUDE
  
  defp init_altitude_mode(state) do
    state
  end
  
  
  defp do_altitude_mode(state = %State{ap_altitude_mode: :off}) do
    state
  end
  
end
