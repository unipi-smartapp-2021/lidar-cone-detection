import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.multioutput import MultiOutputRegressor
from sklearn.svm import SVC, SVR
from sklearn.metrics import mean_squared_error as mse, mean_absolute_error as mae
import pickle


points = pd.read_csv("interpolation_data/centres.csv").to_numpy()

def train_and_test(dataset, title):
    x,y = dataset
    X_train, X_test, y_train, y_test = train_test_split(x, y, test_size=0.05, shuffle=True)


    if y_test.ndim > 1:
        classifier = SVR(kernel="poly", degree=3, C=0.1, epsilon=1e-2)
        svr = MultiOutputRegressor(classifier)
    else:
        svr = SVR(kernel="poly", degree=3, C=0.1, epsilon=1e-2)

    svr.fit(X_train, y_train)

    # Save to file in the current working directory
    pkl_filename = "interpolation_models/{}_svr.pkl".format(title)
    with open(pkl_filename, 'wb') as file:
        pickle.dump(svr, file)

    # Load from file
    with open(pkl_filename, 'rb') as file:
        svr = pickle.load(file)

    y_pred = svr.predict(X_test)

    print("predicted: ", y_pred)
    print("gtruth: ", y_test)
    print("mse {} mae {}\n".format(mse(y_pred, y_test), mae(y_pred, y_test)))



x_data = (points[:,0].reshape(-1, 1), points[:,3])
y_data = (points[:,1].reshape(-1, 1), points[:,4])
z_data = (points[:,2].reshape(-1, 1), points[:,5])
xyz_data = (points[:,:3],points[:,3:])
datasets = {"x axis": x_data, "y_axis":y_data, "z_axis": z_data, "xyz": xyz_data}
for key, value in datasets.items():


    train_and_test(value, key)