defmodule Autopilot.LoaderSupervisor do
  @moduledoc """
  Supervises:
  - XPlane.Instance server for discovering and monitoring X-Plane instances
  - A loader that creates processes for every XPlane instance
  - A dynamic supervisor for the created processes
  """
  
  
  use Supervisor
  
  
  def start_link(_) do
    Supervisor.start_link(__MODULE__, [], name: __MODULE__)
  end

  
  def init(_arg) do
    children = [
      {XPlane.Instance, []},
      {Autopilot.Loader, name: AutopilotLoader},
      {DynamicSupervisor, strategy: :one_for_one, name: AutopilotSupervisor}
    ]

    Supervisor.init(children, strategy: :one_for_all)
  end
  
end
