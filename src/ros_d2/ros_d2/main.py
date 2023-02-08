# -*- coding: utf-8 -*-
import subprocess
import time
from pathlib import Path

import click

from ros_d2.export import export


@click.group()
def cli():
    pass


@cli.command("export", help="Export ROS 2 node graph to D2")
@click.option(
    "--verbose",
    is_flag=True,
    default=False,
    help="Make the diagram more verbose with topic types and more.",
)
@click.argument("output", type=str)
def export_command(output: str, verbose: bool):
    output_file = Path(output)

    # If the output_file is not a .d2 file, add the extension.
    if output_file.suffix != ".d2":
        output_file = output_file.with_suffix(".d2")

    export(output_file, verbose)


@cli.command(
    "render", help="Find all .d2 files in the current directory and render them"
)
@click.option(
    "--layout",
    default="dagre",
    help="The d2 layout type",
)
def render_command(layout: str):
    d2_files = Path(".").glob("**/*.d2")
    d2_files_list = list(d2_files)
    click.echo(f"D2 files found: {len(d2_files_list)}")
    for d2_file in d2_files_list:
        # Execute d2 render command
        command = ["d2", "--layout", layout, str(d2_file)]
        click.echo(f"Running: {' '.join(command)}")
        subprocess.run(command)


@cli.command(
    "export-launch-files",
    help="Finds all your .launch.py files, runs them and exports the resulting graph",
)
@click.option(
    "--wait",
    default=2,
    help="How long to wait for the launch file to start before exporting the graph",
)
def export_launch_files_command(wait: int):
    launch_files = Path(".").glob("**/*.launch.py")
    launch_files_list = list(launch_files)
    click.echo(f"ROS launch files found: {len(launch_files_list)}")
    for launch_file in launch_files_list:
        # Execute d2 render command
        launch_file_name = launch_file.stem
        # replace .launch.py with .d2
        output_file = launch_file.with_name(launch_file_name + ".d2")
        command_launch = [
            "ros2",
            "launch",
            str(launch_file),
        ]
        click.echo(f"Running: {' '.join(command_launch)}")
        # Run this command in a subprocess so we can end the process later
        process = subprocess.Popen(command_launch)
        # Sleep for a bit so the process has time to start
        time.sleep(wait)

        command_export = [
            "ros_d2",
            "export",
            str(output_file),
            "--verbose",
        ]
        click.echo(f"Running: {' '.join(command_export)}")
        subprocess.run(command_export)

        # Kill the process
        process.kill()


if __name__ == "__main__":
    cli()
