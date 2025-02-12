import asyncio
import omni.kit.asset_converter as converter
import os
import argparse

def progress_callback(current_step: int, total: int):
    # Show progress
    print(f"{current_step} of {total}")

async def convert(input_asset_path, output_asset_path):
    task_manager = converter.get_instance()
    task = task_manager.create_converter_task(input_asset_path, output_asset_path, progress_callback)
    success = await task.wait_until_finished()
    if not success:
        detailed_status_code = task.get_status()
        detailed_status_error_string = task.get_error_message()
        print(f"Conversion failed: {detailed_status_error_string} (Status code: {detailed_status_code})")
    else:
        print(f"Conversion successful: {output_asset_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert assets using Omni asset converter.")
    parser.add_argument("input_asset_path", type=str, nargs='?', 
                        default=os.path.join(os.getcwd(), "input", "example.obj"),
                        help="Path to the input asset file.")
    parser.add_argument("output_asset_path", type=str, nargs='?', 
                        default=os.path.join(os.getcwd(), "output", "example.usd"),
                        help="Path to the output asset file.")

    args = parser.parse_args()

    # Ensure the output directory exists
    os.makedirs(os.path.dirname(args.output_asset_path), exist_ok=True)

    # Run the conversion
    asyncio.run(convert(args.input_asset_path, args.output_asset_path))
    