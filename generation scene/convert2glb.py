import os
import asyncio
from omni.kit.asset_converter import get_instance

async def convert(input_asset_path, output_asset_path, progress_callback):
    task_manager = get_instance()
    task = task_manager.create_converter_task(input_asset_path, output_asset_path, progress_callback)
    success = await task.wait_until_finished()
    if not success:
        print(f"Failed to convert {input_asset_path}: {task.get_error_message()}")
    else:
        print(f"Successfully converted {input_asset_path} to {output_asset_path}")

def progress_callback(current_step, total):
    
    progress = (current_step / total) * 100
    bar = '#' * int(progress / 2) + '-' * (50 - int(progress / 2))
    print(f"\r[{bar}] {progress:.2f}% Complete", end="")

async def batch_convert(directory, output_dir):
    
	await convert(directory, output_dir, lambda current, total: progress_callback(i, total_files))

directory = ""
output_dir = ""


loop = asyncio.get_event_loop()
loop.run_until_complete(batch_convert(directory, output_dir))