# Import modules.
import numpy as np

# Function to multiply list of matrices, until a stop point.
def mult_range(mat_list, start=0, stop=None):

    # Check if a stop value was specified.
    if stop == None:
        stop = len(mat_list)

    # Multiply matrices up to stop point.
    mult_list = mat_list[start:stop]
    arr = np.array(mult_list)

    # Define output.
    if len(mult_list) < 2:
        return arr[0]
    else:
        return np.linalg.multi_dot(arr)