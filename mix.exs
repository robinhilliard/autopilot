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
      mod: {Autopilot.Application,[]},
      registered: [:autopilot]
    ]
  end

  # Run "mix help deps" to learn about dependencies.
  defp deps do
    [
      {:xplane, "~> 0.5.0", hex: :elixplane},
      {:ex_doc, "~> 0.19", only: :dev, runtime: false},
      {:dialyzex, "~> 1.2.0", only: :dev}
    ]
  end
end
