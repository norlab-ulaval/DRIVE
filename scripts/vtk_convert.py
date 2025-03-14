import vtk

def convert_vtk_version(input_file, output_file):
    # Attempt to read the file using vtkGenericDataObjectReader
    reader = vtk.vtkGenericDataObjectReader()
    reader.SetFileName(input_file)
    reader.Update()

    # Check if the file was read successfully
    data = reader.GetOutput()
    if data is None:
        raise ValueError(f"Failed to read the input file: {input_file}")
    
    # Identify the type of the VTK data
    if isinstance(data, vtk.vtkPolyData):
        writer = vtk.vtkPolyDataWriter()
    elif isinstance(data, vtk.vtkStructuredGrid):
        writer = vtk.vtkStructuredGridWriter()
    elif isinstance(data, vtk.vtkUnstructuredGrid):
        writer = vtk.vtkUnstructuredGridWriter()
    elif isinstance(data, vtk.vtkImageData):
        writer = vtk.vtkStructuredPointsWriter()
    else:
        raise TypeError(f"Unsupported VTK data type: {type(data)}")

    # Write the file in VTK 3.0 format
    writer.SetFileName(output_file)
    writer.SetInputData(data)
    writer.SetFileTypeToASCII()  # Optional: Ensures ASCII output
    writer.Write()

    print(f"File successfully converted to VTK 3.0 format: {output_file}")

input_vtk_file = "/home/william/Desktop/map.vtk"
output_vtk_file = "/home/william/Desktop/output_legacy.vtk"

# Example usage
try:
    convert_vtk_version(input_vtk_file, output_vtk_file)
except Exception as e:
    print(f"Error: {e}")
