import os

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
from scipy.stats import entropy, wasserstein_distance, wilcoxon
from sklearn.metrics.pairwise import linear_kernel


def get_all_data(
    ISM,
    DIRECTION_AS_TAG,
    selected_features_dict,
):
    DEVICE = "caretronic"
    DETREND = True
    FS = 52
    WINDOW_SIZE = 256
    WINDOW_OVERLAP = 256

    selected_features = selected_features_dict["selected_features"]
    selected_features_name = selected_features_dict["selected_features_name"]

    if ISM:
        ISM = "_m"
    else:
        ISM = ""

    if DETREND:
        DETREND = "_detrend"
    else:
        DETREND = ""

    if DIRECTION_AS_TAG:
        dname = "direction"
    else:
        dname = "position"
    path_to_save = f"results/{selected_features_name}{ISM}/{dname}/"
    appendex = f"{selected_features_name}{ISM}{dname}"
    if not os.path.exists(path_to_save):
        os.makedirs(path_to_save)
    df_o = pd.read_csv(
        f"../features/{DEVICE}_{FS}_v{ISM}_{WINDOW_SIZE}_{WINDOW_OVERLAP}{DETREND}.csv"
    )
    df_o = df_o[df_o["subject"] != 6]
    print(df_o.shape)

    def euclidean_distance(x, y):
        """Euclidean distance metrics calculates the distance between two vectors"""
        x = x.mean(axis=0).to_numpy()
        y = y.mean(axis=0).to_numpy()
        return np.linalg.norm(x - y)

    def cosine_similarity(x, y):
        x_mean = x.mean(axis=0).to_numpy()
        y_mean = y.mean(axis=0).to_numpy()

        # x_mean = x_mean / np.linalg.norm(x_mean)
        # y_mean = y_mean / np.linalg.norm(y_mean)

        dot_product = np.dot(x_mean, y_mean)
        norm_x = np.linalg.norm(x_mean)
        norm_y = np.linalg.norm(y_mean)

        return dot_product / (norm_x * norm_y)

    def mahalanobis(x, y):
        """Mahalanobis distance metrics calculates the distance between two vectors"""
        x_cov = np.cov(x, rowvar=False)
        y_cov = np.cov(y, rowvar=False)
        x_avg = np.mean(x, axis=0)
        y_avg = np.mean(y, axis=0)

        x_minus_y = x_avg - y_avg
        pooled_cov = (x_cov + y_cov) / 2
        inv_covmat = np.linalg.inv(pooled_cov)

        left_term = np.dot(x_minus_y, inv_covmat)
        mahalanobis_distance = np.dot(left_term, x_minus_y.T)
        return np.sqrt(np.abs(mahalanobis_distance))

    def calculate_wasserstein(x, y):
        x = np.array(x)
        y = np.array(y)  # TODO: ali je potrebno pretvoriti v HISTOGRAM?
        dd = []
        for i in range(x.shape[1]):
            w = wasserstein_distance(x[:, i], y[:, i])
            dd.append(w)
        return np.median(dd)

    def calculate_mmd(x, y):
        K_xx = linear_kernel(x, Y=None)
        K_yy = linear_kernel(y, Y=None)
        K_xy = linear_kernel(x, Y=y)
        mmd = np.mean(K_xx) + np.mean(K_yy) - 2 * np.mean(K_xy)
        return mmd

    def calculate_jensen_shannon(x, y):
        # Convert the Pandas DataFrames to NumPy arrays

        x_array = x.to_numpy()
        y_array = y.to_numpy()

        if x_array.shape[0] > y_array.shape[0]:
            x_array = x_array[: y_array.shape[0], :]
        elif x_array.shape[0] < y_array.shape[0]:
            y_array = y_array[: x_array.shape[0], :]

        # Compute the probability distributions for each dimension in x and y
        p_x = x_array / np.sum(
            x_array, axis=1, keepdims=True
        )  # Normalized probabilities for each data point in x
        p_y = y_array / np.sum(
            y_array, axis=1, keepdims=True
        )  # Normalized probabilities for each data point in y

        # Compute the average distributions (mean of probabilities)
        p_avg = 0.5 * (p_x + p_y)

        # Compute the Jensen-Shannon divergence for each data point
        js_divergences = []
        for i in range(x_array.shape[0]):
            q_x = p_x[i]
            q_y = p_y[i]
            m = 0.5 * (p_avg[i] + q_x + q_y)
            js_divergence = 0.5 * (
                entropy(p_avg[i], m) + entropy(q_x, m) + entropy(q_y, m)
            )
            js_divergences.append(js_divergence)

        # Return the array of Jensen-Shannon divergences
        return np.array(js_divergences).mean()

    df_o.reset_index(drop=True, inplace=True)
    todrop = [
        "tag",
        "subject",
        "position",
        "rotated",
        "height",
        "weight",
        "age",
        "gender",
        "direction",
    ]
    todrop_filtered = [t for t in todrop if t in df_o.columns]

    matrix_0 = []
    matrix_1 = []
    subjlist = []
    thresholds = []
    for subj in np.sort(df_o["subject"].unique()):
        df_do = df_o.query(f"subject=={subj}")  # and position==1")
        x = df_do.drop(todrop_filtered, axis=1)
        x = x.dropna(how="any", axis=1)
        x = x.dropna(how="any", axis=0)
        y = df_do.loc[x.index][todrop_filtered]
        xx = x.copy()[selected_features]

        if DIRECTION_AS_TAG:
            tag0_th_d0 = xx[
                (y["tag"] == 0) & (y["direction"] == 0) & (y["position"] == 0)
            ]
            tag0_th_d1 = xx[
                (y["tag"] == 0) & (y["direction"] == 0) & (y["position"] == 1)
            ]

            tag0_test = xx[(y["tag"] == 0) & (y["direction"] == 1)]
            tag1_test = xx[(y["tag"] == 1) & (y["direction"] == 1)]
        else:
            tag0_th_d0 = xx[
                (y["tag"] == 0) & (y["position"] == 1) & (y["direction"] == 0)
            ]
            tag0_th_d1 = xx[
                (y["tag"] == 0) & (y["position"] == 1) & (y["direction"] == 1)
            ]

            tag0_test = xx[(y["tag"] == 0) & (y["position"] == 0)]
            tag1_test = xx[(y["tag"] == 1) & (y["position"] == 0)]

        ###############################################################################################
        # ## COMBINING THE DATSETS AND THEN NORMALIZING AND STANDARDIZING
        tag0_train = pd.concat((tag0_th_d0, tag0_th_d1), axis=0)
        tag0tr_tag0ts = pd.concat((tag0_train, tag0_test), axis=0)
        tag0tr_tag1ts = pd.concat((tag0_train, tag1_test), axis=0)

        # # # Combined normalization
        tag0tr_tag0ts = (tag0tr_tag0ts - tag0tr_tag0ts.min(axis=0)) / (
            tag0tr_tag0ts.max(axis=0) - tag0tr_tag0ts.min(axis=0)
        )
        tag0tr_tag1ts = (tag0tr_tag1ts - tag0tr_tag1ts.min(axis=0)) / (
            tag0tr_tag1ts.max(axis=0) - tag0tr_tag1ts.min(axis=0)
        )

        tag0tr_tag0ts = tag0tr_tag0ts.dropna(how="any", axis=1)
        tag0tr_tag1ts = tag0tr_tag1ts.dropna(how="any", axis=1)

        tag0_train = tag0tr_tag0ts.iloc[: len(tag0_train)]
        tag0_th_d0 = tag0_train.iloc[: len(tag0_th_d0)]
        tag0_th_d1 = tag0_train.iloc[len(tag0_th_d0) :]

        tag1_test = tag0tr_tag1ts.iloc[len(tag0_train) :]
        tag0_test = tag0tr_tag0ts.iloc[len(tag0_train) :]

        tag1_test = tag1_test.dropna(how="any", axis=1)
        tag0_test = tag0_test.dropna(how="any", axis=1)

        threshold_mahalanobis = mahalanobis(tag0_th_d0, tag0_th_d1)
        threshold_cosine = 1 - cosine_similarity(tag0_th_d0, tag0_th_d1)
        threshold_euclidean = euclidean_distance(tag0_th_d0, tag0_th_d1)
        threshold_wasserstein = calculate_wasserstein(tag0_th_d0, tag0_th_d1)
        threshold_mmd = calculate_mmd(tag0_th_d0, tag0_th_d1)
        threshold_js = calculate_jensen_shannon(tag0_th_d0, tag0_th_d1)
        # threshold_manova=calculate_MANOVA(tag0_th_d0,tag0_th_d1)[4]

        thresholds.append(
            [
                threshold_cosine,
                threshold_mahalanobis,
                threshold_euclidean,
                threshold_wasserstein,
                threshold_mmd,
                threshold_js,
            ]
        )

        cosine_similarity_metric_0 = 1 - cosine_similarity(tag0_train, tag0_test)
        mahalanobis_metric_0 = mahalanobis(tag0_train, tag0_test)
        euclidean_metric_0 = euclidean_distance(tag0_train, tag0_test)
        wasserstein_metric_0 = calculate_wasserstein(tag0_train, tag0_test)
        mmd_distance_metric_0 = calculate_mmd(tag0_train, tag0_test)
        js_metric_0 = calculate_jensen_shannon(tag0_train, tag0_test)
        # manova_distance_metric_0=calculate_MANOVA(tag0_train,tag0_test)[4]

        cosine_similarity_metric_1 = 1 - cosine_similarity(tag0_train, tag1_test)
        mahalanobis_metric_1 = mahalanobis(tag0_train, tag1_test)
        euclidean_metric_1 = euclidean_distance(tag0_train, tag1_test)
        wasserstein_metric_1 = calculate_wasserstein(tag0_train, tag1_test)
        mmd_distance_metric_1 = calculate_mmd(tag0_train, tag1_test)
        js_metric_1 = calculate_jensen_shannon(tag0_train, tag1_test)
        # manova_distance_metric_1=calculate_MANOVA(tag0_train,tag1_test)[4]

        subjlist.append(subj)
        matrix_0.append(
            [
                cosine_similarity_metric_0,
                mahalanobis_metric_0,
                euclidean_metric_0,
                wasserstein_metric_0,
                mmd_distance_metric_0,
                js_metric_0,
            ]
        )
        matrix_1.append(
            [
                cosine_similarity_metric_1,
                mahalanobis_metric_1,
                euclidean_metric_1,
                wasserstein_metric_1,
                mmd_distance_metric_1,
                js_metric_1,
            ]
        )

    matrix_0 = np.array(matrix_0).T
    matrix_1 = np.array(matrix_1).T
    diff = matrix_1 / matrix_0
    thresholds = np.array(thresholds).T

    # calculate wilcoxon for each metric and plot the result to the sns heatmap
    wilcoxonList = []
    for i in range(len(matrix_0)):
        wilcoxonList.append(wilcoxon(matrix_0[i], matrix_1[i])[1])

    wilcoxonList = np.array(wilcoxonList)
    metrics = [
        "Kosinusna",
        "Mahalanobisova",
        "Euklidska",
        "Warssesteinova",
        "MMD",
        "Jensen-Shannon",
    ]

    plt.figure(figsize=(8, 4))
    sns.heatmap(
        wilcoxonList.reshape(1, -1),
        annot=True,
        fmt=".3f",
        cmap="YlGnBu",
        cbar=False,
        xticklabels=metrics,
        yticklabels=["p vrednost"],
    )
    plt.title(f"Wilcoxon p vrednosti za {selected_features_name}")
    plt.savefig(f"{path_to_save}wilcox_{appendex}.png", dpi=1000)

    from sklearn.metrics import accuracy_score

    # Assuming you have the ground truth labels in the variable y_true
    # y_true = ...

    threshold_multipliers = np.linspace(0, 3, 60)  # or any range you prefer
    max_dicts = {}

    plt.figure(figsize=(8, 5))

    # Iterate over all the metrics
    for i, metric_name in enumerate(metrics):
        accuracies = []
        maxacc = 0
        # Iterate over all threshold multipliers
        for thresh_mult in threshold_multipliers:
            # Calculate the threshold
            threshold = thresholds[i] * thresh_mult
            # Check whether each sample passes the threshold
            predictions = np.where(
                (matrix_0[i] < threshold) & (matrix_1[i] > threshold), 1, 0
            )
            # Compute the accuracy
            accuracy = accuracy_score(np.ones_like(predictions), predictions)
            accuracies.append(accuracy)
            if maxacc < accuracy:
                maxacc = accuracy
                max_dicts[metric_name] = thresh_mult

        # Plot accuracy over threshold multipliers
        plt.plot(threshold_multipliers, accuracies, label=f"{metric_name} razdalja")

    plt.xlabel("Konstanta za množenje z pragom")
    plt.ylabel("Točnost")
    plt.title(f"Značilnosti določene z {selected_features_name}")
    plt.legend(loc="upper right")
    plt.savefig(f"{path_to_save}thresholds_{appendex}.png", dpi=1000)

    max_list = np.array([np.round(v, 2) for k, v in max_dicts.items()])

    ###metrike
    plt.figure(figsize=(12, 4))
    sns.heatmap(
        thresholds,
        annot=True,
        cbar=False,
        yticklabels=[
            "Kosinusna",
            "Mahalanobisova",
            "Euklidska",
            "Warssesteinova",
            "MMD",
            "Jensen-Shannon",
        ],
        xticklabels=subjlist,
        vmin=0,
        vmax=20,
    )
    plt.title(f"Prag za {selected_features_name}")
    plt.xlabel("Osebe")
    plt.ylabel("Metrika razdalje")
    plt.figure(figsize=(12, 4))
    sns.heatmap(
        matrix_0,
        annot=True,
        cbar=False,
        yticklabels=[
            "Kosinusna razdalja",
            "Mahalanobisova",
            "euclidean_distance",
            "Warssesteinova",
            "MMD",
            "Jensen-Shannon",
        ],
        xticklabels=subjlist,
    )

    plt.title(f"Razdalje negativnih primerov za {selected_features_name}")
    plt.xlabel("Osebe")
    plt.ylabel("Metrika razdalje")
    plt.savefig(f"{path_to_save}metrics_0_{appendex}.png", dpi=1000)

    plt.figure(figsize=(12, 4))
    sns.heatmap(
        matrix_1,
        annot=True,
        cbar=False,
        yticklabels=[
            "Kosinusna razdalja",
            "Mahalanobisova",
            "euclidean_distance",
            "Warssesteinova",
            "MMD",
            "Jensen-Shannon",
        ],
        xticklabels=subjlist,
    )
    plt.title(f"Razdalje pozitivnih primerov za {selected_features_name}")
    plt.xlabel("Osebe")
    plt.ylabel("Metrika razdalje")
    plt.savefig(f"{path_to_save}metrics_1_{appendex}.png", dpi=1000)

    plt.figure(figsize=(12, 4))
    sns.heatmap(
        diff,
        annot=True,
        cbar=False,
        yticklabels=[
            "Kosinusna razdalja",
            "Mahalanobisova",
            "euclidean_distance",
            "Warssesteinova",
            "MMD",
            "Jensen-Shannon",
        ],
        xticklabels=subjlist,
    )
    plt.title(
        f"Razmerje razdalj med pozitiivnimi in negativnimi primeri za {selected_features_name}"
    )
    plt.xlabel("Osebe")
    plt.ylabel("Metrika razdalje")
    plt.savefig(f"{path_to_save}diff_{appendex}.png", dpi=1000)
    thresholds_multiplied = thresholds * max_list.reshape(-1, 1)

    checked = np.logical_and(
        matrix_0 < thresholds_multiplied, matrix_1 > thresholds_multiplied
    )
    plt.figure(figsize=(12, 4))
    sns.heatmap(
        checked,
        annot=True,
        cbar=False,
        yticklabels=[
            "Kosinusna razdalja",
            "Mahalanobisova",
            "euclidean_distance",
            "Warssesteinova",
            "MMD",
            "Jensen-Shannon",
        ],
        xticklabels=subjlist,
        vmin=0,
        vmax=1,
    )
    plt.title(
        f"Prag se nahaja med pozitivnimi in negativnimi razdaljami pri {selected_features_name}"
    )
    plt.xlabel("Osebe")
    plt.ylabel("Metrika razdalje")
    plt.savefig(f"{path_to_save}checked_{appendex}.png", dpi=1000)
    print(checked.sum(axis=1) / checked.shape[1])

    TP = thresholds_multiplied < matrix_1
    TN = thresholds_multiplied > matrix_0
    FP = thresholds_multiplied > matrix_1
    FN = thresholds_multiplied < matrix_0
    metrics = [
        "Kosinusno",
        "Mahalanobisovo",
        "Euklidsko",
        "Warssesteinovo",
        "MMD",
        "Jensen-Shannon",
    ]
    for i, metric_name in enumerate(metrics):
        TP = thresholds_multiplied[i] < matrix_1[i]
        TN = thresholds_multiplied[i] > matrix_0[i]
        FP = thresholds_multiplied[i] > matrix_1[i]
        FN = thresholds_multiplied[i] < matrix_0[i]

        confusion_matrix = np.array([[TP.sum(), FP.sum()], [FN.sum(), TN.sum()]])
        plt.figure()
        sns.heatmap(
            confusion_matrix,
            annot=True,
            cbar=False,
            yticklabels=["Pozitivni ", "Negativni"],
            xticklabels=["Napovedani pozitivni", "Napovedani negativni"],
        )
        plt.title(
            f"Konfuzijska matrika za {metric_name} razdaljo pri {selected_features_name}"
        )
        plt.savefig(f"{path_to_save}_confmat_{metric_name}_{appendex}.png")
    return checked[1]
