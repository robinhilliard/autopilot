defmodule Autopilot.Loader do
  @moduledoc """
  Create Data, Command and Autopilot GenServers for every master
  XPlane instance we can see on the network. Don't do it more than
  once for each instance. The DynamicSupervisor will restart them
  if they crash, and if the Loader or the DynamicSupervisor crash
  our Supervisor will restart both so that they stay in sync.
  """
  
  use GenServer
  
  
  def start_link(_) do
    GenServer.start_link(__MODULE__, [], name: __MODULE__)
  end

  
  def init(_opts) do
    schedule_check()
    {:ok, %{}}
  end

  
  def handle_info(:check, state) do
    schedule_check()
    {
      :noreply,
      Enum.reduce(
        XPlane.Instance.list()
        |> Enum.filter(
          fn inst ->
            case inst do
                 %XPlane.Instance{host: :xplane, role: :master, addr: a} ->
                  !Map.has_key?(state, a)
                _ ->
                  false
            end
          end
        ),
        state,
        fn instance, state ->
          start_autopilot(instance)
          Map.put_new(state, instance.addr, instance)
        end)
    }
  end
  
  
  defp schedule_check do
    Process.send_after(self(), :check, 1000)
  end
  
  
  defp start_autopilot(instance) do
    DynamicSupervisor.start_child(AutopilotSupervisor, {XPlane.Cmd, instance})
    DynamicSupervisor.start_child(AutopilotSupervisor, {XPlane.Data, instance})
  end
  
end
