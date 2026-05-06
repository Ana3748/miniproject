import os
from dotenv import load_dotenv
from huggingface_hub import login, hf_hub_download
from ultralytics import YOLO

def main():
    # Load environment variables
    load_dotenv()
    hf_token = os.getenv("HF_TOKEN")
    if not hf_token:
        raise ValueError("Error: 'HF_TOKEN' not found in .env file.")
    
    # 1. Login to Hugging Face
    login(token=hf_token)
    
    # 2. Download the PyTorch model weights (will load instantly if already cached)
    try:
        model_path = hf_hub_download(
            repo_id="Perception365/VehicleNet-Y26m",
            filename="weights/best.pt"
        )
    except Exception as e:
        print(f"Failed to download the model: {e}")
        return

    # 3. Load standard PyTorch model on CPU
    print("Loading PyTorch model on CPU...")
    model = YOLO(model_path)
    
    # 4. Run inference
    image_source = "traffic.png"
    if os.path.exists(image_source):
        print(f"Running inference on '{image_source}'...")
        # conf=0.15 is calibrated for YOLO26m to catch all 50+ vehicles
        results = model.predict(source=image_source, conf=0.15, save=True, device="cpu")
        results[0].show()
    else:
        print(f"Note: '{image_source}' was not found. Please place it in this folder.")

if __name__ == "__main__":
    main()
