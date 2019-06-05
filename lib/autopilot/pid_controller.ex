defmodule Autopilot.PIDController do
  @moduledoc """
  A PID controller tries to set feedback to setpoint by modulating output
  """
  defstruct ~w[p i d output_min output_max err_sum err_prev time_prev]a
  
  def add_pid(state, feedback_key, setpoint_key, output_key,
        p \\ 0.0, i \\ 0.0, d \\ 0.0, output_min \\ -1.0, output_max \\ 1.0)
      when is_map(state) and is_atom(setpoint_key)
           and is_atom(feedback_key) and is_atom(output_key)
           and is_float(p) and is_float(i) and is_float(d)
           and is_float(output_min) and is_float(output_max) do
    Map.put_new(
      state,
      {:pid, feedback_key, setpoint_key, output_key},
      %__MODULE__{
        p:          p,
        i:          i,
        d:          d,
        output_min: output_min,
        output_max: output_max,
        err_sum:    0.0,
        err_prev:   0.0,
        time_prev:  nil
      }
    )
  end
  
  def set_pid_output({state, time}, feedback_key, setpoint_key, output_key)
      when is_map(state) and is_integer(time)
           and is_atom(feedback_key) and is_atom(setpoint_key) and is_atom(output_key) do
    %__MODULE__{
      p:            p,
      i:            i,
      d:            d,
      output_min:   output_min,
      output_max:   output_max,
      err_sum:      err_sum,
      err_prev:     err_prev,
      time_prev:    time_prev
    } = pid_state = state |> Map.fetch!({:pid, feedback_key, setpoint_key, output_key})
    
    # Calculate error
    feedback = state[feedback_key]
    setpoint = state[setpoint_key]
    err = setpoint - feedback
    err_sum = err_sum + err
    
    cond do
      time_prev != nil and time > time_prev ->
        # Determine error rate, calculate/set output and update PID state
        dt = time - time_prev
        err_dt = (err - err_prev) / dt
        output = (p * err + i * err_sum + d * err_dt) |> min(output_max) |> max(output_min)
        
        {
          %{state |
            output_key => output,
            {:pid, feedback_key, setpoint_key, output_key} =>
              %__MODULE__{pid_state |
                err_sum: err_sum,
                err_prev: err,
                time_prev: time
              }
          },
          time
        }
        
      true ->
        # PID can only accumulate errors and set output once per time step
        # Update PID state so that we can calculate error rate and output next time
        {
          %{state |
            {:pid, feedback_key, setpoint_key, output_key} =>
              %__MODULE__{pid_state |
                err_prev: err,
                time_prev: time
              }
          },
          time
        }
        
    end
    
  end

end
