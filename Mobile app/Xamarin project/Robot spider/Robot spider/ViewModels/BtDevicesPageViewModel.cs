using Plugin.BluetoothClassic.Abstractions;
using Robot_spider.Views;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Diagnostics;
using System.Text;
using Xamarin.Forms;

namespace Robot_spider.ViewModels
{
    public class BtDevicesPageViewModel : BindableBase
    {
        private IBluetoothManagedConnection _connection;
        private IBluetoothAdapter _btAdapter;
        public INavigation _navigation;

        private ContentPage _mainpage = null;

        public ObservableCollection<BluetoothDeviceModel> BondedDevices { get; set; }

        private BluetoothDeviceModel _selectedDevice;
        private BluetoothDeviceModel _selectedDeviceForVM;
        public BluetoothDeviceModel SelectedDevice
        {
            get
            {
                return _selectedDeviceForVM;
            }
            set
            {
                _selectedDeviceForVM = null;
                if (_selectedDevice != value)
                {
                    Connect(value);
                    _selectedDevice = value;
                }
                RaisePropertyChanged();
                //if (SetProperty(ref _selectedDevice, value))
                //    Connect(_selectedDevice);
            }
        }

        public BtDevicesPageViewModel()
        {
            _btAdapter = DependencyService.Resolve<IBluetoothAdapter>();
            if (!_btAdapter.Enabled)
                _btAdapter.Enable();
            BondedDevices = new ObservableCollection<BluetoothDeviceModel>(_btAdapter.BondedDevices);

        }

        public void Connect(BluetoothDeviceModel device)
        {
            if (App.CurrentBluetoothConnection != null)
            {
                App.CurrentBluetoothConnection.Dispose();
                App.CurrentBluetoothConnection = null;
            }
            _connection = _btAdapter.CreateManagedConnection(device);
            _connection.OnStateChanged += Connection_OnStateChanged;
            try
            {
                _connection.Connect();
            }
            catch (Exception e)
            {
                Debug.WriteLine(e.Message);
                return;
            }

        }

        private void Connection_OnStateChanged(object sender, StateChangedEventArgs stateChangedEventArgs)
        {
            if (stateChangedEventArgs.ConnectionState == ConnectionState.Connected)
            {
                App.CurrentBluetoothConnection = _connection;
                Device.BeginInvokeOnMainThread(async () =>
                {
                    if (_mainpage == null)
                        _mainpage = new MainPage();
                    await _navigation.PushModalAsync(_mainpage);
                });
                _selectedDevice = null;
            }
        }
    }
}
