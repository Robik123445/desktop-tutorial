import logging
from pathlib import Path
import click

from cam_slicer.logging_config import setup_logging
from cam_slicer.config.machine_config import MACHINE_PRESETS, get_machine_config

setup_logging()

@click.command(help='Setup wizard for CAM Slicer. Docs: https://example.github.io/cam_slicer/')
@click.option('--lang', type=click.Choice(['en', 'sk']), help='Interface language')
@click.option('--project', 'project_dir', type=click.Path(), help='Project folder')
@click.option('--sample', type=click.Path(exists=False), help='Path to sample file')
@click.option('--profile', type=click.Choice(list(MACHINE_PRESETS.keys())), help='Machine profile')

def cli(lang, project_dir, sample, profile):
    """Run an interactive setup wizard for new users."""
    click.echo('=== CAM Slicer Setup Wizard ===')
    if not lang:
        lang = click.prompt('Select language', type=click.Choice(['en', 'sk']), default='en')
    if not project_dir:
        project_dir = click.prompt('Project folder', default=str(Path.cwd()))
    project_path = Path(project_dir)
    project_path.mkdir(parents=True, exist_ok=True)
    if not sample and click.confirm('Import example file?', default=False):
        sample = 'sample.gcode'
    if not profile:
        profile = click.prompt('Machine profile', type=click.Choice(list(MACHINE_PRESETS.keys())), default=list(MACHINE_PRESETS.keys())[0])
    config = {
        'language': lang,
        'project': str(project_path),
        'sample': sample,
        'profile': profile,
    }
    logging.info('Wizard result: %s', config)
    click.echo(f'Selected profile: {profile}')
    try:
        get_machine_config(profile)
    except KeyError:
        click.echo('Unknown profile selected.')
    click.echo('Setup complete.')
    return config

if __name__ == '__main__':
    cli()
