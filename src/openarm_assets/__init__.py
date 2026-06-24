"""MuJoCo models for the OpenArm robot."""

from pathlib import Path

__version__ = "0.1.0"
MODELS_DIR = Path(__file__).parent / "models"

# Just OpenArm for now
AVAILABLE_MODELS = ["openarm"]

def get_model_path(name: str = "openarm") -> Path:
    """Get the path to a MuJoCo model ZML file."""

    model_dir = MODELS_DIR / name

    if not model_dir.exists():
        raise FileNotFoundError(f"Model '{name}' not found. Available models: {AVAILABLE_MODELS}")
    
    xml_condidates = [
        model_dir / f"{name}.xml",
        model_dir / "scene.xml",
    ]

    for xml_path in xml_condidates:
        if xml_path.exists():
            return xml_path
        
    raise FileNotFoundError(f"No XML file found in {model_dir}. Tried {[p.name for p in xml_condidates]}")
         