defmodule PIDControllerTest do
  use ExUnit.Case
  import Enum, only: [reduce: 3]
  import Autopilot.PIDController
  alias Autopilot.PIDController, as: PIDController

  test "add_pid defaults" do
    %{{:pid, :feedback, :setpoint, :output} => %PIDController{
      p: p,
      i: i,
      d: d,
      output_min: output_min,
      output_max: output_max
    }} = add_pid(%{}, :feedback, :setpoint, :output)
    assert p == 0.0
    assert i == 0.0
    assert d == 0.0
    assert output_min == -1.0
    assert output_max == 1.0
  end
  
   test "add_pid non-defaults" do
    %{{:pid, :feedback, :setpoint, :output} => %PIDController{
      p: p,
      i: i,
      d: d,
      output_min: output_min,
      output_max: output_max
    }} = add_pid(
      %{},
      :feedback,
      :setpoint,
      :output,
      p: 0.1, i: 0.2, d: 0.3, output_min: 4.0, output_max: 5.0)
    assert p == 0.1
    assert i == 0.2
    assert d == 0.3
    assert output_min == 4.0
    assert output_max == 5.0
  end
  
  test "Output not set without time increase" do
    start_state = %{
      feedback: 1.0,
      setpoint: 0.0,
      output: 0.0,
    }
    |> add_pid(
         :feedback,
         :setpoint,
         :output,
         p: 0.1, i: 0.0, d: 0.0)
    
    end_state = reduce(1..10, start_state,
      fn _, state ->
        {s1, _} = {state, 0} |> set_pid_output(:feedback, :setpoint, :output)
        %{s1 | feedback: s1.feedback + s1.output}
      end
    )
    
    assert end_state.feedback == 1.0
    
  end
  
  test "P converges" do
    start_state = %{
      feedback: 1.0,
      setpoint: 0.0,
      output: 0.0,
    }
    |> add_pid(
         :feedback,
         :setpoint,
         :output,
         p: 0.1, i: 0.0, d: 0.0)
    
    end_state = reduce(1..67, start_state,
      fn time, state ->
        {s1, _} = {state, time} |> set_pid_output(:feedback, :setpoint, :output)
        %{s1 | feedback: s1.feedback + s1.output}
      end
    )
    
    assert abs(end_state.feedback) < 0.001
    
  end
  
  test "PI copes with drift" do
    start_state = %{
      feedback: 1.0,
      setpoint: 0.0,
      output: 0.0,
    }
    |> add_pid(
         :feedback,
         :setpoint,
         :output,
         p: 0.1, i: 0.09, d: 0.0)
    
    end_state = reduce(1..67, start_state,
      fn time, state ->
        {s1, _} = {state, time} |> set_pid_output(:feedback, :setpoint, :output)
        %{s1 | feedback: s1.feedback + s1.output + 0.01}
      end
    )
    
    assert abs(end_state.feedback) < 0.001
    
  end
  
  test "PID converges more quickly than PI and copes with drift" do
    start_state = %{
      feedback: 1.0,
      setpoint: 0.0,
      output: 0.0,
    }
    |> add_pid(
         :feedback,
         :setpoint,
         :output,
         p: 0.1, i: 0.09, d: -0.03)
    
    end_state = Enum.reduce(1..66, start_state,
      fn time, state ->
        {s1, _} = {state, time} |> set_pid_output(:feedback, :setpoint, :output)
        %{s1 | feedback: s1.feedback + s1.output + 0.01}
      end
    )
    
    assert abs(end_state.feedback) < 0.001
    
  end
  
  test "PID output min-max limits obeyed" do
    start_state = %{
      limits_exceeded: [],
      feedback: 1.0,
      setpoint: 0.0,
      output: 0.0,
    }
    |> add_pid(
         :feedback,
         :setpoint,
         :output,
         p: 0.1, i: 0.0, d: 0.0,
         output_min: -0.05, output_max: 0.05)
    
    end_state = reduce(1..67, start_state,
      fn time, state ->
        {s1, _} = {state, time} |> set_pid_output(:feedback, :setpoint, :output)
        cond do
          s1.output >= -0.05 and s1.output <= 0.05 ->
            %{s1 | feedback: s1.feedback + s1.output}
          true ->
            %{s1 |
              limits_exceeded: [s1.output | s1.limits_exceeded],
              feedback: s1.feedback + s1.output}
        end
      end
    )
    
    assert length(end_state.limits_exceeded) == 0
    
  end
  
end
