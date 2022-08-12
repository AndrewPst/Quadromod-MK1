using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Text;
using Xamarin.Forms;
using Robot_spider.Models;
using Plugin.BluetoothClassic.Abstractions;
using Robot_spider.Views;
using System.Collections.ObjectModel;

namespace Robot_spider.ViewModels
{
    public enum SelectedMode_t
    {
        StaticMode,
        DynamicMode,
        RotationMode,
        OffsetMode
    }

    public struct ModItem
    {
        public SelectedMode_t mode;
        public string name;
        public string key;
        public Func<string> getDataToSend;
    }

    public class ModeItemViewModel : BindableBase
    {
        private ModItem item;

        public SelectedMode_t Mode
        {
            get => item.mode;
            set => SetProperty(ref item.mode, value);
        }

        private bool _isSelected;
        public bool IsSelected
        {
            get => _isSelected;
            set
            {
                SetProperty(ref _isSelected, value);
            }
        }

        public string Name
        {
            get => item.name;
        }

        public string Key { get => item.key; }

        public Func<string> getDataToSend { get => item.getDataToSend; }

        public ModeItemViewModel(SelectedMode_t mode, string name, string key, Func<string> action)
        {
            item.mode = mode;
            item.name = name;
            item.key = key;
            item.getDataToSend = action;
        }
    }

    public class MainPageViewModel : BindableBase
    {
        public INavigation _navigation;
        public DelegateCommand sendCommand { get; set; }

        private bool _isUpdateNeeding = false;
        private bool _isSetupUpdateNeeding = false;
        private bool _isPlayCustomActionNeeding = false;

        public ObservableCollection<ModeItemViewModel> modeItems { get; set; }
        public ObservableCollection<string> customActions { get; set; }

        private string[] customActionKeys = new string[]
        {
            "D1",
            "D2",
            "GP",
            "UC",
            "PH",
            "AT"
        };

        public DelegateCommand DisconnectCommand { get; set; }
        public DelegateCommand PlayCommand { get; set; }
        public MainPageViewModel()
        {
            App.OnConnectionStateChanged += (o, x) => Subscribe(x);

            DisconnectCommand = new DelegateCommand(() => Disconnect());
            PlayCommand = new DelegateCommand(() => _isPlayCustomActionNeeding = true);

            if (App.CurrentBluetoothConnection != null)
            {
                Subscribe(App.CurrentBluetoothConnection);
            }

            modeItems = new ObservableCollection<ModeItemViewModel>()
            {
                new ModeItemViewModel(SelectedMode_t.DynamicMode, "Dynamic", "D", getDataFromDynamicMode),
                new ModeItemViewModel(SelectedMode_t.StaticMode, "Static", "S", getDataFromStaticMode),
                new ModeItemViewModel(SelectedMode_t.RotationMode, "Rotation", "R", getDataFromRotationMode),
                new ModeItemViewModel(SelectedMode_t.OffsetMode, "Offset", "O", getDataFromOffsetMode),
            };

            customActions = new ObservableCollection<string>()
            {
                "Dance 1",
                "Dance 2",
                "Give me a pow",
                "undercut",
                "pull the housing",
                "attack"
            };

            SelectedItem = modeItems[0];
            Device.StartTimer(new TimeSpan(0, 0, 0, 0, 200), TimerCallBack);
        }

        private void Subscribe(IBluetoothManagedConnection connection)
        {
            if (connection == null) return;
            if (connection.ConnectionState == ConnectionState.Connected)
            {
                App.CurrentBluetoothConnection.OnStateChanged += CurrentBluetoothConnection_OnStateChanged;
            }
        }

        private string getSetupData()
        {
            return $"#|P|{ServoSpeed}|{AccValue + 1}|{ZOffset}";
        }

        private string getDataFromDynamicMode()
        {
            string value = '#'.ToString() + '|'.ToString() + "D" + '|'.ToString() + JLeftD.ToString() + '|' + (360 - JLeftA).ToString() + '|' + JRightX.ToString();
            return value;
        }
        private string getDataFromStaticMode()
        {
            string value = string.Empty;
            if (Math.Abs(JLeftX) > 30)
            {
                value = '#'.ToString() + "|" + "S" + "|" + "R" + "|" + JLeftX.ToString();
            }
            else
            {
                value = '#'.ToString() + "|" + "S" + "|" + "M" + "|" + JLeftY.ToString();
            }
            return value;
        }
        private string getDataFromRotationMode()
        {
            return $"#|R|{JLeftX}|{JLeftY}|{JRightX}";
        }
        private string getDataFromOffsetMode()
        {
            string value = $"#|O|{JLeftX}|{JLeftY}|{JRightY}";
            return value;
        }

        private bool TimerCallBack()
        {
            if (_isPlayCustomActionNeeding)
            {
                sendData(getStrCustomAction());
                _isPlayCustomActionNeeding = false;
            }
            else if (_isSetupUpdateNeeding)
            {
                string value = getSetupData();
                sendData(value);
                _isSetupUpdateNeeding = false;
            }
            else if (_isUpdateNeeding)
            {
                string value = SelectedItem.getDataToSend();
                sendData(value);
                _isUpdateNeeding = false;
            }
            return true;
        }

        private string getStrCustomAction()
        {
            return $"#|C|{customActionKeys[_selectedCustomAction]}";
        }

        private void CurrentBluetoothConnection_OnStateChanged(object sender, Plugin.BluetoothClassic.Abstractions.StateChangedEventArgs stateChangedEventArgs)
        {
            if (stateChangedEventArgs.ConnectionState == ConnectionState.ErrorOccured)
            {
                Device.BeginInvokeOnMainThread(async () =>
                {
                    try
                    {
                        await _navigation.PopModalAsync();
                    }
                    catch { }
                });
            }
        }

        private void sendData(string text)
        {
            if (App.CurrentBluetoothConnection != null)
            {
                Memory<byte> array = Encoding.UTF8.GetBytes(text + '\0');
                App.CurrentBluetoothConnection.Transmit(array);
            }
        }

        private void Disconnect()
        {
            App.CurrentBluetoothConnection.Dispose();
            App.CurrentBluetoothConnection = null;
            Device.BeginInvokeOnMainThread(async () =>
            {
                await _navigation.PopModalAsync();
            });
        }

        private int _selectedCustomAction = 0;
        public int SelectedCUstomAction
        {
            get => _selectedCustomAction;
            set
            {
                SetProperty(ref _selectedCustomAction, value);
            }
        }

        private int _jLeftX;
        public int JLeftX
        {
            get { return _jLeftX; }
            set { if (SetProperty(ref _jLeftX, value)) _isUpdateNeeding = true; }
        }

        private int _jLeftD;
        public int JLeftD
        {
            get { return _jLeftD; }
            set { if (SetProperty(ref _jLeftD, value)) _isUpdateNeeding = true; }
        }

        private int _jLeftA;
        public int JLeftA
        {
            get { return _jLeftA; }
            set { if (SetProperty(ref _jLeftA, value)) _isUpdateNeeding = true; }
        }

        private int _jLeftY;
        public int JLeftY
        {
            get { return _jLeftY; }
            set { if (SetProperty(ref _jLeftY, value)) _isUpdateNeeding = true; }
        }

        private int _jRightX;
        public int JRightX
        {
            get { return _jRightX; }
            set { if (SetProperty(ref _jRightX, value)) _isUpdateNeeding = true; }
        }

        private int _jRightD;
        public int JRightD
        {
            get { return _jRightD; }
            set { if (SetProperty(ref _jRightD, value)) _isUpdateNeeding = true; }
        }

        private int _jRightA;
        public int JRightA
        {
            get { return _jRightA; }
            set { if (SetProperty(ref _jRightA, value)) _isUpdateNeeding = true; }
        }

        private int _jRightY;
        public int JRightY
        {
            get { return _jRightY; }
            set { if (SetProperty(ref _jRightY, value)) _isUpdateNeeding = true; }
        }


        private double _value = 0;
        public double Value
        {
            get => _value;
            set
            {
                _value = Math.Round(value);
                SelectedItem = modeItems[(int)_value > modeItems.Count ? modeItems.Count : (int)_value < 0 ? 0 : (int)_value];
                RaisePropertyChanged();
            }
        }

        private double _accValue = 1.8;
        public double AccValue
        {
            get => _accValue;
            set
            {
                if (SetProperty(ref _accValue, value))
                    _isSetupUpdateNeeding = true;
            }
        }

        private int _servoSpeed = 65;
        public int ServoSpeed
        {
            get => _servoSpeed;
            set
            {
                if (SetProperty(ref _servoSpeed, value))
                    _isSetupUpdateNeeding = true;
            }
        }

        private int _zOffset = 40;
        public int ZOffset
        {
            get => _zOffset;
            set
            {
                if (SetProperty(ref _zOffset, value))
                    _isSetupUpdateNeeding = true;
            }
        }

        private ModeItemViewModel _selectedItem;
        public ModeItemViewModel SelectedItem
        {
            get => _selectedItem;
            set
            {
                if (_selectedItem != null)
                    _selectedItem.IsSelected = false;
                _selectedItem = value;
                _selectedItem.IsSelected = true;
                RaisePropertyChanged();
            }
        }

    }
}
