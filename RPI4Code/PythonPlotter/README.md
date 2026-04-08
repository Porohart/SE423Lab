# Python plotter


## Setup

We will install / setup the dependency using `uv` (a great new fast python manager)

(the most up to date instruction are [here](https://docs.astral.sh/uv/#__tabbed_1_2) if these don't work)

macOS / Linux
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

```powershell
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
```

Then it is very simple, making sure that you are inside the `PythonPlotter` folder (using `cd`)

```bash
uv sync
```

and done!

## Run

To run it is also very simple!

```bash
uv run main.py
```

and done!