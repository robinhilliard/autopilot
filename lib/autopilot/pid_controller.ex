defmodule Autopilot.PIDController do
  @moduledoc """
  A PID controller tries to set a feedback value to a setpoint value by modulating an output value.
  
  The output value is calculated as the sum of the following three components multiplied by their
  corresponding coefficients:
  
  - P(roportional): How big is the error?
  - I(ntegral): What is the cumulative sum of all errors (positive and negative) to date?
  - D(erivative): How quickly is the error changing?
  
  Several well known techniques for tuning these values can be found on the web.
  
  Typically pid controllers are chained together, with the output of one controller acting as the
  setpoint of another controller. To facilitate composable chains of pid controllers the `add_pid()`
  and `set_pid_output()` functions work with a map containing a mix of feedback, setpoint, output
  and pid parameters, which allows pid calculations to access input/output values and update their
  own state as required. Individual pid controllers are identified by unique feedback, setpoint and
  output key combinations.
  
  Users are responsible for providing a time parameter to `set_pid_output()`, used to calculate
  error change rates. The function will not accumulate errors or set the output for a controller
  unless the time has increased since the last call for that controller.
  
  ## Example

      iex> state = %{feedback: 1.0, setpoint: 0.0, output: 0.0}
      %{feedback: 1.0, output: 0.0, setpoint: 0.0}
      iex> state = state |> add_pid(:feedback, :setpoint, :output, p: 0.1, i: 0.09, d: -0.03)
      %{
        :feedback => 1.0,
        :output => 0.0,
        :setpoint => 0.0,
        {:pid, :feedback, :setpoint, :output} => %Autopilot.PIDController{
          d: -0.03,
          err_prev: 0.0,
          err_sum: 0.0,
          i: 0.09,
          output_max: 1.0,
          output_min: -1.0,
          p: 0.1,
          time_prev: :pos_infinity
        }
      }
      iex> Enum.reduce(1..66, state,
      ...>   fn time, state ->
      ...>     {s1, _} = {state, time} |> set_pid_output(:feedback, :setpoint, :output)
      ...>     %{s1 | feedback: s1.feedback + s1.output + 0.01}
      ...>   end)[:feedback]
      -7.551632489127894e-4
    
  1. Create a state map with a feedback that we are trying to set to 0.0 by adjusting an output used by
  a simple simulation
  2. Add pid details to the state, specifying the map keys containing the feedback, setpoint and output values
  and some PID coefficients developed for this example by trial and error
  3. Iterate to allow the controller to take control of the simulation:
  
      - Pass the current state and a monotonically increasing time counter in a tuple to `set_pid_output()`
        which calculates an output and stores it in the `:output` key of our state map
      - Simulate a system being controlled, which adds the output of the pid to its position (indicated by
        the feedback value) and drifts a little in the positive direction (perhaps due to wind or gravity) and
        updates the value of our state's `:feedback` key
  4. Return the feedback after 66 iterations, which is close to the desired setpoint of 0.0 at ~ -0.00075

  """
  
  
  defstruct [
    p: 0.0,                   # Proportional coefficient
    i: 0.0,                   # Integral coefficient
    d: 0.0,                   # Derivative coefficient
    output_min: -1.0,         # Clip output
    output_max: 1.0,
    err_sum: 0.0,             # Sum of errors for Integral
    err_prev: 0.0,            # To calculate error change rate
    time_prev: :pos_infinity  # numbers < atoms term ordering hack
  ]
  
  @type t :: %__MODULE__{
    p: float, i: float, d: float,
    output_min: float, output_max: float,
    err_sum: float, err_prev: float,
    time_prev: number | atom
  }
  
  
  @spec add_pid(map, atom, atom, atom, [{atom, float}]) :: map
  def add_pid(state, feedback_key, setpoint_key, output_key, params \\ [])
      when is_map(state)
           and is_atom(setpoint_key)
           and is_atom(feedback_key)
           and is_atom(output_key)
           and is_list(params) do
    
    Map.put_new(
      state,
      {:pid, feedback_key, setpoint_key, output_key},
      struct(%__MODULE__{}, params)
    )
    
  end
  
  
  @spec set_pid_output({map, number}, atom, atom, atom) :: {map, number}
  def set_pid_output({state, time}, feedback_key, setpoint_key, output_key)
      when is_map(state)
           and is_number(time)
           and is_atom(feedback_key)
           and is_atom(setpoint_key)
           and is_atom(output_key) do
    
    pid_key = {:pid, feedback_key, setpoint_key, output_key}
    pid_state = Map.fetch!(state, pid_key)
    feedback = Map.fetch!(state, feedback_key)
    setpoint = Map.fetch!(state, setpoint_key)
    
    {new_pid_state, output} = step_pid(pid_state, time, feedback, setpoint)
    
    case output do
      nil ->
        {%{state | pid_key => new_pid_state}, time}
        
      _ ->
        {%{state | output_key => output, pid_key => new_pid_state}, time}
        
    end
    
  end
  
  
  @spec step_pid(t, number, float, float) :: {t, float | nil}
  
  defp step_pid(
    pid_state = %__MODULE__{
      p: p, i: i, d: d,
      err_sum: err_sum, err_prev: err_prev, time_prev: time_prev,
      output_min: output_min, output_max: output_max},
    time, feedback, setpoint)
      when time > time_prev
           and is_float(feedback)
           and is_float(setpoint) do
    
    err = setpoint - feedback
    err_sum = err_sum + err
    dt = time - time_prev
    err_dt = (err - err_prev) / dt
    output = (p * err + i * err_sum + d * err_dt)
             |> min(output_max)
             |> max(output_min)

    {struct(pid_state, err_sum: err_sum, err_prev: err, time_prev: time), output}
    
  end
  
  defp step_pid(
    pid_state, time, feedback, setpoint)
      when is_number(time)
           and is_float(feedback)
           and is_float(setpoint) do
    
      {struct(pid_state, err_prev: setpoint - feedback, time_prev: time), nil}
      
  end
  
end
