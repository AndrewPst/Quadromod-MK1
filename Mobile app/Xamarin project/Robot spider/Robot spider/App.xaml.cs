using System;
using Xamarin.Forms;
using Xamarin.Forms.Xaml;
using Robot_spider.Views;
using Plugin.BluetoothClassic;
using Plugin.BluetoothClassic.Abstractions;

namespace Robot_spider
{
    public partial class App : Application
    {
        private static IBluetoothManagedConnection _currentConnection;
        public static IBluetoothManagedConnection CurrentBluetoothConnection
        {
            get => _currentConnection;
            set
            {
                _currentConnection = value;
                OnConnectionStateChanged?.Invoke(App.Current, value);
            }
        }

        public static event EventHandler<IBluetoothManagedConnection> OnConnectionStateChanged;
        public App()
        {
            InitializeComponent();
            NavigationPage navigationPage = new NavigationPage(new BtDevicesPage());
            INavigation navigation = navigationPage.Navigation;


            MainPage = navigationPage;
        }

        protected override void OnStart()
        {
        }

        protected override void OnSleep()
        {
            //_currentConnection.Dispose();
        }

        protected override void OnResume()
        {
        }
    }
}
