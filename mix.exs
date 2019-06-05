defmodule Autopilot.MixProject do
  use Mix.Project

  def project do
    [
      app: :autopilot,
      version: "0.1.0",
      elixir: "~> 1.8",
      start_permanent: Mix.env() == :prod,
      deps: deps()
    ]
  end

  # Run "mix help compile.app" to learn about applications.
  def application do
    [
      extra_applications: [:logger],
      registered: [Autopilot]
    ]
  end

  # Run "mix help deps" to learn about dependencies.
  defp deps do
    [
      {:xplane, "~> 0.4.0", hex: :elixplane}
    ]
  end
end