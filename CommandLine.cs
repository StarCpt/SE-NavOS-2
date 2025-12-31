using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace IngameScript
{
    public class CommandLine
    {
        public GPS? Gps { get; private set; }
        public int Count => args.Count;

        private List<string> args = new List<string>();

        private static readonly char[] charSpace = { ' ' };

        public CommandLine(string command)
        {
            ParseArgs(command);
        }

        private void ParseArgs(string command)
        {
            GPS gps;
            if (GPS.TryParse(command, out gps))
            {
                Gps = gps;
                int gpsStrIndex = command.IndexOf(Gps.Value.OriginalString);
                string[] argsBeforeGps = command.Substring(0, gpsStrIndex).Split(charSpace, StringSplitOptions.RemoveEmptyEntries);
                string[] argsAfterGps = command.Substring(gpsStrIndex + Gps.Value.OriginalString.Length).Split(charSpace, StringSplitOptions.RemoveEmptyEntries);

                args.AddRange(argsBeforeGps);
                args.Add(Gps.Value.OriginalString);
                args.AddRange(argsAfterGps);
            }
            else
            {
                args.AddRange(command.Split(charSpace, StringSplitOptions.RemoveEmptyEntries));
            }
        }

        public string this[int index, bool lowerCase = false] => args.IsValidIndex(index) ? (lowerCase ? args[index].ToLower() : args[index]) : null;
        public bool Matches(int index, string str, bool ignoreCase = true)
        {
            if (!args.IsValidIndex(index))
            {
                return false;
            }

            return args[index].Equals(str, ignoreCase ? StringComparison.CurrentCultureIgnoreCase : StringComparison.CurrentCulture);
        }
    }
}
