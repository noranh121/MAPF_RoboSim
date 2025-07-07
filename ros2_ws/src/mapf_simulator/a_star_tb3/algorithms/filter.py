import cv2
import numpy as np
import matplotlib.pyplot as plt
import argparse

def display_and_save(image, title, save_path=None):
    plt.imshow(image, cmap='gray')
    plt.title(title)
    plt.axis('off')
    plt.show()
    if save_path:
        cv2.imwrite(save_path, image)

def calculate_power_spectrum(fourier):
    power_spectrum = np.log(1 + np.abs(fourier))
    min = np.min(power_spectrum)
    power_spectrum = (power_spectrum - min) / (np.max(power_spectrum) - min)
    return power_spectrum

def ideal(type, shape, cutoff):
    rows, cols = shape
    center_row, center_col = rows // 2, cols // 2
    filter = np.zeros(shape, dtype=np.float32)
    for i in range(rows):
        for j in range(cols):
            distance = np.sqrt((i - center_row)**2 + (j - center_col)**2)
            if distance <= cutoff:
                filter[i, j] = 1
    if type == 'H':
        return 1 - filter
    return filter


def butterworth(type, shape, cutoff, order=2):
    """Creates a Butterworth low-pass filter."""
    rows, cols = shape
    center_row, center_col = rows // 2, cols // 2
    filter = np.zeros(shape, dtype=np.float32)
    for i in range(rows):
        for j in range(cols):
            distance = np.sqrt((i - center_row)**2 + (j - center_col)**2)
            filter[i, j] = 1 / (1 + (distance / cutoff)**(2 * order))
    if type == 'H':
        return 1 - filter
    return filter


def gaussian(type, shape, cutoff):
    rows, cols = shape
    center_row, center_col = rows // 2, cols // 2
    filter = np.zeros(shape, dtype=np.float32)
    for i in range(rows):
        for j in range(cols):
            distance = np.sqrt((i - center_row)**2 + (j - center_col)**2)
            filter[i, j] = np.exp(-(distance**2) / (2 * (cutoff**2)))
    if type == 'H':
        return 1 - filter
    return filter


def main():
    parser = argparse.ArgumentParser(description="Apply frequency domain filtering to an image", add_help=False)
    parser.add_argument('--help', action='help', default=argparse.SUPPRESS, help='Show help message and exit')
    parser.add_argument("-i", "--input_img", required=True, help="Path to the input image")
    parser.add_argument("-s", "--spectrum_img", help="Path to save the power spectrum image")
    parser.add_argument("-f", "--filter_type", required=True, choices=['L', 'H'], help="Filter type: (L/H) low or high pass")
    parser.add_argument("-m", "--filter_mode", required=True, choices=['I', 'B', 'G'], help="Generated filter mode: (I/B/G) Ideal, Butterworth, or Gaussian")
    parser.add_argument("-h", "--filter_img", help="Path to save the filter image.")
    parser.add_argument("-j", "--filtered_img", help="Path to save the power spectrum of the filtered image in frequency domain")
    parser.add_argument("-o", "--final_img", required=True, help="Path to save the filtered image in the spatial domain")
    # diameter in case they forgot to require this
    # parser.add_argument("-d", "--diameter", type=float, required=True, help="Diameter (cutoff frequency) for the filter.")
 
    args = parser.parse_args()

    input_image_path = args.input_img
    spectrum_path = args.spectrum_img
    filter_type = args.filter_type
    filter_mode = args.filter_mode
    # cutoff_diameter = args.diameter
    filter_path = args.filter_img
    filtered_spectrum_path = args.filtered_img
    output_image_path = args.final_img

    # 1. Load an image and converts it to the frequency domain.
    img = cv2.imread(input_image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"Error: Could not open or find the image at {input_image_path}")
        return

    f_transform = np.fft.fft2(img)
    f_shift = np.fft.fftshift(f_transform)

    # 2. Display/Save the power spectrum.
    power_spectrum = calculate_power_spectrum(f_shift)
    display_and_save(power_spectrum, "Power Spectrum", spectrum_path)

    # diameter in case they want us to calcualte it 
    rows, cols = img.shape
    min_dimension = min(rows, cols)
    cutoff_diameter = min_dimension / 8

    # 3. Compute a low pass (L) or High pass (H) filter.
    rows, cols = img.shape
    filter_shape = (rows, cols)
    filter = np.zeros(filter_shape, dtype=np.float32)

    if filter_mode == 'I':
        filter = ideal(filter_type ,filter_shape, cutoff_diameter)
    elif filter_mode == 'B':
        filter = butterworth(filter_type, filter_shape, cutoff_diameter)
    elif filter_mode == 'G':
        filter = gaussian(filter_type, filter_shape, cutoff_diameter)
        

    # 4. Display/Save the computed filter
    display_and_save(filter, f"{filter_type}-Pass {filter_mode} Filter (D={cutoff_diameter})", filter_path)

    # 5. Apply the computed filter to the input image on the frequency domain
    filtered_f_shift = f_shift * filter

    # 6. Display/Save the filtered image (in frequency domain as power spectrum)
    filtered_power_spectrum = calculate_power_spectrum(filtered_f_shift)
    display_and_save(filtered_power_spectrum, "Filtered Power Spectrum", filtered_spectrum_path)

    # 7. Convert the filtered image back to the spatial domain
    f_ishift = np.fft.ifftshift(filtered_f_shift)
    img_back = np.fft.ifft2(f_ishift)
    img_back = np.abs(img_back)
    img_back = cv2.normalize(img_back, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)

    # 8. Display/Save the filtered image
    display_and_save(img_back, "Filtered Image", output_image_path)

if __name__ == "__main__":
    main()