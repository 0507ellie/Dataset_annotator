import os

def create_text_files_from_filenames(folder_path):
    """
    Creates empty .txt files with the same names as all files in the specified folder.
    
    Args:
        folder_path (str): Path to the folder containing the files to copy names from.
    """
    # Get all files in the specified folder
    try:
        files = [f for f in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, f))]
    except FileNotFoundError:
        print(f"Error: The folder '{folder_path}' does not exist.")
        return
    except PermissionError:
        print(f"Error: Permission denied when accessing '{folder_path}'.")
        return
    
    if not files:
        print("No files found in the specified folder.")
        return
    
    # Create a subfolder to store the new text files (optional)
    output_folder = os.path.join(folder_path, "text_files")
    os.makedirs(output_folder, exist_ok=True)
    
    created_count = 0
    
    for filename in files:
        # Split the filename and extension
        name, ext = os.path.splitext(filename)
        
        # Create new filename with .txt extension
        new_filename = f"{name}.txt"
        new_filepath = os.path.join(output_folder, new_filename)
        
        try:
            # Create empty text file
            with open(new_filepath, 'w') as f:
                pass
            created_count += 1
        except IOError as e:
            print(f"Error creating file {new_filename}: {e}")
    
    print(f"Successfully created {created_count} text files out of {len(files)} original files.")
    print(f"Text files are saved in: {output_folder}")

if __name__ == "__main__":
    # Get folder path from user input
    folder_path = input("Enter the path to the folder: ").strip()
    
    # Normalize the path (removes any trailing slashes, etc.)
    folder_path = os.path.normpath(folder_path)
    
    create_text_files_from_filenames(folder_path)