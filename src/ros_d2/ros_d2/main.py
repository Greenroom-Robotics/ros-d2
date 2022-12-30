# -*- coding: utf-8 -*-
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


if __name__ == "__main__":
    cli()
