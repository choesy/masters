import numpy as np
from scipy.stats import skew, kurtosis
from scipy.signal import get_window, detrend
from scipy.fft import rfft



window= [0.0, 0.00015059, 0.00060227, 0.00135477, 0.00240764, 0.00376023,
                            0.00541175, 0.00736118, 0.00960736, 0.01214894, 0.01498437, 0.01811197,
                            0.02152983, 0.02523591, 0.02922797, 0.0335036, 0.03806023, 0.04289512,
                            0.04800535, 0.05338785, 0.05903937, 0.0649565, 0.07113569, 0.07757322,
                            0.08426519, 0.09120759, 0.09839623, 0.10582679, 0.11349478, 0.12139558,
                            0.12952444, 0.13787647, 0.14644662, 0.15522973, 0.16422053, 0.17341357,
                            0.18280336, 0.1923842, 0.20215034, 0.2120959, 0.22221488, 0.2325012,
                            0.24294862, 0.25355092, 0.26430163, 0.27519435, 0.28622246, 0.29737934,
                            0.30865827, 0.32005247, 0.33155507, 0.34315914, 0.35485765, 0.3666436,
                            0.3785099, 0.39044937, 0.40245485, 0.41451904, 0.42663476, 0.43879467,
                            0.45099142, 0.4632177, 0.47546616, 0.4877294, 0.5, 0.5122706,
                            0.5245338, 0.53678226, 0.54900855, 0.5612053, 0.5733652, 0.5854809,
                            0.59754515, 0.6095506, 0.62149006, 0.6333564, 0.6451423, 0.65684086,
                            0.66844493, 0.6799475, 0.6913417, 0.7026207, 0.71377754, 0.72480565,
                            0.73569834, 0.7464491, 0.75705135, 0.7674988, 0.7777851, 0.7879041,
                            0.79784966, 0.8076158, 0.81719667, 0.8265864, 0.8357795, 0.84477025,
                            0.8535534, 0.86212355, 0.8704756, 0.8786044, 0.88650525, 0.8941732,
                            0.90160376, 0.90879244, 0.9157348, 0.92242676, 0.9288643, 0.9350435,
                            0.94096065, 0.9466122, 0.95199466, 0.95710486, 0.96193975, 0.9664964,
                            0.970772, 0.9747641, 0.97847015, 0.98188806, 0.98501563, 0.9878511,
                            0.9903926, 0.9926388, 0.99458826, 0.9962398, 0.9975924, 0.99864525,
                            0.99939775, 0.99984944, 1.0, 0.99984944, 0.99939775, 0.99864525,
                            0.9975924, 0.9962398, 0.99458826, 0.9926388, 0.9903926, 0.9878511,
                            0.98501563, 0.98188806, 0.97847015, 0.9747641, 0.970772, 0.9664964,
                            0.96193975, 0.95710486, 0.95199466, 0.9466122, 0.94096065, 0.9350435,
                            0.9288643, 0.92242676, 0.9157348, 0.90879244, 0.90160376, 0.8941732,
                            0.88650525, 0.8786044, 0.8704756, 0.86212355, 0.8535534, 0.84477025,
                            0.8357795, 0.8265864, 0.81719667, 0.8076158, 0.79784966, 0.7879041,
                            0.7777851, 0.7674988, 0.75705135, 0.7464491, 0.73569834, 0.72480565,
                            0.71377754, 0.7026207, 0.6913417, 0.6799475, 0.66844493, 0.65684086,
                            0.6451423, 0.6333564, 0.62149006, 0.6095506, 0.59754515, 0.5854809,
                            0.5733652, 0.5612053, 0.54900855, 0.53678226, 0.5245338, 0.5122706,
                            0.5, 0.4877294, 0.47546616, 0.4632177, 0.45099142, 0.43879467,
                            0.42663476, 0.41451904, 0.40245485, 0.39044937, 0.3785099, 0.3666436,
                            0.35485765, 0.34315914, 0.33155507, 0.32005247, 0.30865827, 0.29737934,
                            0.28622246, 0.27519435, 0.26430163, 0.25355092, 0.24294862, 0.2325012,
                            0.22221488, 0.2120959, 0.20215034, 0.1923842, 0.18280336, 0.17341357,
                            0.16422053, 0.15522973, 0.14644662, 0.13787647, 0.12952444, 0.12139558,
                            0.11349478, 0.10582679, 0.09839623, 0.09120759, 0.08426519, 0.07757322,
                            0.07113569, 0.0649565, 0.05903937, 0.05338785, 0.04800535, 0.04289512,
                            0.03806023, 0.0335036, 0.02922797, 0.02523591, 0.02152983, 0.01811197,
                            0.01498437, 0.01214894, 0.00960736, 0.00736118, 0.00541175, 0.00376023,
                            0.00240764, 0.00135477, 0.00060227, 0.00015059] # window type: hann, window length: 256, sampling frequency: 52

CF_FS =52                                # sampling frequency of the signal
CF_M =256                                # window length
CF_D= 128                                # overlap length
CF_BUFFER_SIZE =256                      # length of the array
CF_L= (CF_BUFFER_SIZE - CF_M) / CF_D + 1 # number of windows in the signal
CF_SAMPLING_FREQENCY =52
CF_TIMESTEP= 1 / CF_SAMPLING_FREQENCY

def mean_value(arr):
    """Ok"""
    return np.mean(arr)

def standard_deviation(arr):
    """Ok"""
    return np.std(arr)

def coefficient_of_variation(arr):
    """Ok"""
    mean=np.mean(arr)
    if mean == 0:
        mean=1
    return np.std(arr) / mean

def num_of_zero_crossings(arr):
    """Ok"""
    arr = np.array(arr) # Ensuring that the input is a numpy array
    return np.sum((arr[:-1] * arr[1:]) < 0)

def calc_amplitude(arr):
    """Ok"""
    return np.max(arr) - np.min(arr)


def compute_AC_velocity(arr, timestep):
    """Ok"""
    arr = np.array(arr) # convert to numpy array if it's not
    dT = np.empty_like(arr) 
    dT[0] = 1.0
    dT[1:] = timestep
    return np.sum(arr * dT)

def calc_sum_per_component(arr, timestep):
    """Ok"""
    arr = np.array(arr) # convert to numpy array if it's not
    dT = np.empty_like(arr)
    dT[0] = 1.0
    dT[1:] = timestep
    return np.sum(np.abs(arr) * dT)


def abs_mean_value(arr):
    """Ok"""
    return np.mean(np.abs(arr))

def calc_mean_crossing_rate(arr):
    """Ok"""
    mean = np.mean(np.abs(arr))
    shifted_arr = arr - mean
    return np.sum(shifted_arr[:-1] * shifted_arr[1:] < 0)


def cmpfunc(a, b):
    """Ok"""
    return 1 if a >= b else -1


def median_index(l, r):
    """Ok"""
    n = r - l + 1
    n = (n + 1) // 2 - 1
    return n + l


def calculate_IQR(a):
    """Ok"""
    Q1 = np.percentile(a, 25)
    Q3 = np.percentile(a, 75)
    IQR = Q3 - Q1
    return IQR


def calc_entropy(arr):
    """Ok"""
    arr=np.array(arr)
    sum_arr = np.sum(np.abs(arr))
    arr = arr/sum_arr #normalize the values
    arr = arr[arr > 0] # keep only positive values to avoid log(0)
    entropy = -np.sum(arr * np.log(arr))
    return entropy

def absolute_value_complex(tmpdata):
    """Ok"""
    flipped_FFT = fft_and_flip_array(tmpdata)
    niza=np.array(flipped_FFT)
    # Calculate the absolute values of the complex numbers
    re=np.real(niza)

    im=np.imag(niza)
    absolute_array = np.sqrt(np.square(re) + np.square(im))
    # Calculate the energy
    energy_feat = np.sum(np.square(absolute_array)) / len(absolute_array) *2

    entropy = calc_entropy(absolute_array)

    return energy_feat, entropy


def sample_frequencies(n, nfft, fs):
    """Ok"""
    return np.arange(n) * fs / nfft

def detrend_func(arr):
    """Ok"""
    arr=np.array(arr)
    return np.subtract(arr,np.mean(arr))

def window_data(arr1,arr2):
    """Ok"""
    arr1=np.array(arr1)
    arr2=np.array(arr2)
    return arr1*arr2


def calc_skewness_and_kurtosis(arr):
    """Ok"""
    skewness = skew(arr)
    kurtosis_value = kurtosis(arr)

    return skewness, kurtosis_value


def conjugate_multiply(niza, scale):
    """Ok-ish prvi in zadnji od 256 elementov se razlikujeta ma np"""
    niza=np.array(niza)
    # Convert the 1D real-imaginary array to a 2D array of complex numbers
    complex_arr = niza[::2] + 1j * niza[1::2]
    
    # Calculate the squares of the absolute values of the complex numbers (equivalent to conjugate multiplication)
    new_list = np.square(np.abs(complex_arr)) * scale

    # Double the values for all elements except the first and last
    new_list[1:-1] *= 2

    return new_list

def PSD(input_arr,FS):
    input_arr=np.array(input_arr)
    window = get_window("hann", len(input_arr))
    scale = 1 / (FS * np.sum(window ** 2))

    fft_inst = np.fft.rfft(input_arr*window)  # Perform FFT
    psd = np.square(np.abs(fft_inst))* scale # Calculate PSD
    return psd

def welch_method(niza, out_length):
    niza=np.array(niza)

    # Scaling factor based on sampling frequency and window type
    # Note: You need to set the window type to match your C code
    window_type = 'hann'
    window = get_window(window_type, CF_M)
    scale = 1 / (CF_FS * np.sum(window ** 2))

    # Number of segments
    l = (len(niza) - CF_M) // CF_D + 1

    # Remove linear trend from data
    niza = detrend_func(niza)

    # Create array to hold final output
    final_arr = np.empty((l, out_length))

    # Iterate over each segment
    for i in range(l):
        segment = niza[i * CF_D:i * CF_D + CF_M]

        # Remove linear trend from segment
        segment = detrend_func(segment)

        # Apply window to segment
        segment *= window

        # Perform FFT on windowed segment
        segment_fft_output = rfft(segment)

        # Multiply FFT results by their conjugates (equivalent to calculating absolute values and squaring)
        # Note: We're only taking the first `out_length` values because `rfft` returns an array of length `CF_M // 2 + 1`
        final_arr[i, :] = np.abs(segment_fft_output[:out_length]) ** 2 * scale

    return final_arr


def fft_and_flip_array(input_arr):
    # Perform FFT
    fft_output = np.fft.rfft(input_arr)
    fft_output[-1] = fft_output[1]
    fft_output[1] = 0  # Set second element to 0
    
    return fft_output


def total_fft_sum(arr):
    sum_ = np.sum(np.square(arr))
    return sum_

def normalized_squared_sum_N_elements(arr, start, stop, total_fft_sum):
    sum_ = np.sum(np.square(arr[start:stop]))
    return sum_ / total_fft_sum

def fill_bin(input_arr, fft_sum):
    bin_arr = np.zeros(10)
    for i in range(0, 45, 5):
        bin_arr[i // 5] = normalized_squared_sum_N_elements(input_arr, i, i+5, fft_sum)

    bin_arr[9] = normalized_squared_sum_N_elements(input_arr, 45, 129, fft_sum)
    return bin_arr



def find_N_max_values(input_arr, n):
    temp_arr = input_arr.copy()
    result_values = np.zeros(n)
    result_indicies = np.zeros(n, dtype=int)
    sample_freq = np.linspace(0, CF_SAMPLING_FREQENCY/2, 129)
    for i in range(n):
        result_indicies[i] = np.argmax(temp_arr)
        result_values[i] = temp_arr[result_indicies[i]]
        temp_arr[result_indicies[i]] = np.finfo(float).min

    return sample_freq[result_indicies]

def standardize_features(features, mean_vals, std_vals):
    features = np.array(features)
    mean_vals = np.array(mean_vals)
    std_vals = np.array(std_vals)
    standardized_features = (features - mean_vals) / std_vals
    return standardized_features.tolist()


def calc_v_features_from_multiple(arr,FS):
    f=np.zeros((len(arr),37))
    for i in range(len(arr)):
        f[i]=calc_v_features(arr[i],FS)
    return f

def calc_v_features_from_xyz(arrx,arry,arrz,FS):
    fx=calc_v_features_from_multiple(arrx,FS)
    fy=calc_v_features_from_multiple(arry,FS)
    fz=calc_v_features_from_multiple(arrz,FS)
    f=np.concatenate((fx,fy,fz),axis=1)
    return f
    

  
def calc_v_features(array,FS):
    """
    Names of the features are in the names list
    Names: mean_value, standard_deviation, coefficient_of_variation, abs_mean_value, num_of_zero_crossings, amplitude,
    AC_velocity, sum_per_component, mean_crossing_rate, entropy, IQR, skewness, kurtosis, f_energy, f_entropy, f_bin_1,
    f_bin_2, f_bin_3, f_bin_4, f_bin_5, f_bin_6, f_bin_7, f_bin_8, f_bin_9, f_bin_10, f_skewness, f_kurtosis, f_max_1,
    f_max_2, f_max_3, f_max_4, f_max_5, f_max_6, f_max_7, f_max_8, f_max_9, f_max_10
    
    
    """
    features=np.zeros(37)
    features[0] = mean_value(array)
    features[1] = standard_deviation(array)
    features[2] = coefficient_of_variation(array)
    features[3] = abs_mean_value(array)
    features[4] = num_of_zero_crossings(array)
    features[5] = calc_amplitude(array)
    features[6] = compute_AC_velocity(array, 1/FS)
    features[7] = calc_sum_per_component(array,1/FS)
    features[8] = calc_mean_crossing_rate(array)
    features[9] = calc_entropy(array)
    features[10] = calculate_IQR(array)
    features[11], features[12] = calc_skewness_and_kurtosis(array)

    features[13],features[14]=absolute_value_complex(array)
    Pxx_den = PSD(array,FS)
    fft_sum = total_fft_sum(Pxx_den)
    
    features[15:25]=fill_bin(Pxx_den, fft_sum)
    features[25], features[26] = calc_skewness_and_kurtosis(Pxx_den)
    features[27:37]=find_N_max_values(Pxx_den,10)
    
  
    return features
    


if __name__=="__main__":
    tmpdata=[-1.32009659,  0.78899369,  0.19826683, -0.17095574,  1.50890873,
        -0.03512237,  0.02951333,  0.70637045, -1.63652624, -0.22210952,
        0.34816699,  1.8153743 ,  0.17569488, -1.46300936,  0.71714331,
        0.01873775,  0.61514081, -0.58409327, -0.69415832, -0.33859855,
        0.8844943 ,  0.97343996,  0.20069523,  0.34244851, -0.65994231,
        -0.62194378,  2.00485212,  0.82753871,  0.50521242,  0.44793862,
        1.00538525, -0.23367753, -0.31773938,  1.50961298,  0.15736217,
        -0.18239001,  1.05103275,  1.37442847, -1.733193  , -1.23137755,
        -0.60993552, -0.58034526,  0.83621373, -0.37073605,  0.03147029,
        0.9679239 , -0.41641082,  2.38358321,  0.66907286,  1.75999775,
        -0.34616439,  0.27802573, -2.25572708,  0.62609733, -1.04418612,
        -0.39207424, -0.42035396, -0.686716  , -0.61601256, -1.46090206,
        0.07662948,  0.35623383,  1.51972358, -1.10862898,  0.70061608,
        -0.16570193, -0.99219895,  1.3477293 ,  0.42886681,  2.04756007,
        0.21378822, -1.258062  , -1.66564881,  1.42215837, -0.95076328,
        3.10608296,  1.00861992,  1.21580999, -0.96354393, -0.360354  ,
        -0.499325  , -0.87888876,  1.07027958,  1.0198242 ,  0.47857531,
        -1.06394571,  0.68503657,  0.56192179,  1.94320834, -0.40594381,
        -0.4550447 ,  1.26080424,  0.01171305, -0.12717974, -0.0879902 ,
        -1.29599734,  2.27494292,  0.06916198,  0.80158078,  1.66474166,
        0.61983782, -0.91553627, -0.50649706,  1.44306476, -0.44880267,
        0.40147842, -1.49751591,  0.32006729, -0.63551155,  0.04200041,
        -0.82351221,  1.2591201 ,  0.3857175 ,  1.34399411,  1.62911687,
        0.60453607, -1.78522058, -0.42930978, -1.0771649 , -1.55268624,
        -0.44764405,  1.0724082 , -1.57878334,  0.2882309 , -0.65679024,
        0.35049811, -0.60282002, -0.71969658, -1.76042138,  0.01940097,
        -0.19691344, -2.03533379,  0.40995238,  1.16550176, -0.86458591,
        -0.92532257,  0.92113147,  2.51109855, -2.21650497, -0.38281852,
        2.26274241, -0.43852408, -0.96726987,  0.11711828,  0.48427898,
        0.15494413, -0.08425578, -0.08813816,  1.27786619,  0.20509855,
        -0.37667071, -0.33390825, -0.56459688, -1.20871418,  1.4896309 ,
        0.06055802, -1.42934516, -0.14272153,  1.78542457, -0.76867844,
        -1.88370437,  0.65923571, -0.53691397,  0.7061336 ,  0.36563932,
        0.62215255,  1.42463021, -0.48258156, -1.44671582,  0.73874866,
        -1.26039337,  0.52727935,  0.05119854, -0.43606811, -0.13618939,
        0.87659386,  0.30916093, -0.40117433,  1.53791421, -0.13897537,
        -2.14940018,  1.0895835 , -0.82951456,  1.99503171, -1.97550036,
        0.39913063, -1.71159449, -1.16770347, -1.15450769,  0.46201692,
        -1.37348519, -0.99165835, -0.64741003, -1.93712842, -1.02396882,
        -0.70667166,  0.97693337,  0.91020606, -1.65500708, -2.07630682,
        0.95762729, -0.63938064,  1.57710868, -0.833656  ,  1.33475811,
        1.57961695,  0.23824983,  1.32368686, -2.01569174,  0.91865996,
        -0.03286638,  0.43115212, -1.18963474, -0.4010064 ,  0.2921477 ,
        -0.0630721 , -0.10574091,  1.13923179, -0.8359432 ,  0.55286354,
        -0.5677964 , -0.93701208,  0.56685532, -1.58046332,  0.41873406,
        0.53879409, -0.93319274,  0.09443291, -0.32488518, -0.67368593,
        0.02211479, -0.19492255,  0.18017895,  0.03064022, -1.0419191 ,
        -0.77958945,  0.67669875, -0.38637206, -0.4780427 , -0.81248569,
        -0.27342431,  0.32592318, -0.17709873,  0.19366026,  1.23915964,
        -0.11999708,  0.62363022, -0.35876786,  0.96268012, -2.31956691,
        -0.38351036,  0.25808993,  0.58202209,  0.85962234,  0.80433584,
        0.09443291]
    
    datavata1=np.random.rand(100,256)
    datavata2=np.random.rand(100,256)
    datavata3=np.random.rand(100,256)
    features=calc_v_features(tmpdata,52)
    print(features)